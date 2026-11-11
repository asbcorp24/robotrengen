#include "BleTransport.h"

#include "Config.h"
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs_mbuf.h>
#include <stdio.h>
#include <string.h>

#if !defined(CONFIG_NIMBLE_ENABLED)
#error This transport uses the bundled Arduino NimBLE BLE library (pioarduino 55.03.312).
#endif

namespace {
const char* phyName(uint8_t phy) {
  switch (phy) {
    case BLE_GAP_LE_PHY_1M: return "1M";
    case BLE_GAP_LE_PHY_2M: return "2M";
    case BLE_GAP_LE_PHY_CODED: return "Coded";
    default: return "unknown";
  }
}
}  // namespace

void BleTransport::begin(uint32_t bootId) {
  bootId_ = bootId;
  ingress_ = xQueueCreate(16, sizeof(BleFragment));
  events_ = xQueueCreate(8, sizeof(Event));
  if (!ingress_ || !events_) abort();
  char name[24];
  snprintf(name, sizeof(name), "Crawler-%06llX", ESP.getEfuseMac() & 0xffffffULL);
  if (!BLEDevice::init(name)) abort();
  const int listenerStatus = ble_gap_event_listener_register(&phyListener_, onGapEvent, this);
  if (listenerStatus != 0) Serial.printf("BLE PHY listener status=%d\n", listenerStatus);
  BLEDevice::setMTU(config::kBleMtu);
  BLEDevice::setSecurityCallbacks(this);
  // Bring-up pairing: encrypted LE Secure Connections, Just Works + bonding.
  // No MITM protection / physical ownership transfer yet. See README.
  BLESecurity::setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  BLESecurity::setCapability(ESP_IO_CAP_NONE);
  BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLESecurity::setKeySize(16);
  BLESecurity::setForceAuthentication(true);

  server_ = BLEDevice::createServer();
  server_->setCallbacks(this);
  server_->advertiseOnDisconnect(false);
  BLEService* service = server_->createService(config::kServiceUuid);
  // Write with response only; ATT success means transport receipt, NOT execution.
  rx_ = service->createCharacteristic(config::kRxUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_ENC);
  rx_->setCallbacks(this);
  tx_ = service->createCharacteristic(config::kTxUuid,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ_ENC);
  tx_->setCallbacks(this);
  // NimBLE creates the CCCD descriptor automatically.
  info_ = service->createCharacteristic(config::kInfoUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_READ_ENC);
  info_->setCallbacks(this);
  service->start();
  server_->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->setName(name);
  advertising->addServiceUUID(config::kServiceUuid);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.printf("BLE: %s, MTU <= %u, encrypted development pairing\n", name, config::kBleMtu);
}

void BleTransport::onConnect(BLEServer* server, ble_gap_conn_desc* p) {
  if (connected_.load()) {
    server->disconnect(p->conn_handle);
    return;
  }
  connectionId_.store(p->conn_handle);
  encrypted_.store(false);
  subscribed_.store(false);
  phyPending_.store(false);
  mtu_.store(23);
  generation_.fetch_add(1);
  restartAdvertising_.store(false);
  connected_.store(true);
  recordEvent({EventKind::Connect, p->conn_handle, 0, 0, false});
  BLEDevice::getAdvertising()->stop();
  // The bundled server starts security after this callback (forceAuthentication).
}
void BleTransport::onDisconnect(BLEServer*, ble_gap_conn_desc* p) {
  if (p->conn_handle != connectionId_.load()) return;
  recordEvent({EventKind::Disconnect, p->conn_handle, 0, 0, false});
  connected_.store(false);
  encrypted_.store(false);
  subscribed_.store(false);
  phyPending_.store(false);
  connectionId_.store(0xffff);
  generation_.fetch_add(1);
  restartAdvertising_.store(true);
}
void BleTransport::onMtuChanged(BLEServer*, ble_gap_conn_desc* p, uint16_t mtu) {
  if (p->conn_handle != connectionId_.load()) return;
  mtu_.store(mtu < 23 ? 23 : (mtu > config::kBleMtu ? config::kBleMtu : mtu));
}
void BleTransport::onAuthenticationComplete(ble_gap_conn_desc* p, int status) {
  if (!p || !connected_.load() || p->conn_handle != connectionId_.load()) return;
  recordEvent({EventKind::Security, p->conn_handle, status,
               static_cast<uint8_t>(p->sec_state.key_size), p->sec_state.encrypted != 0});
  const bool secure = status == 0 && p->sec_state.encrypted && p->sec_state.key_size == 16;
  const bool wasEncrypted = encrypted_.load();
  if (wasEncrypted != secure) {
    encrypted_.store(false);
    generation_.fetch_add(1);
  }
  encrypted_.store(secure);
  if (secure && !wasEncrypted) phyPending_.store(true);
  if (!secure) server_->disconnect(connectionId_.load());
}
void BleTransport::onSubscribe(BLECharacteristic* characteristic, ble_gap_conn_desc* p, uint16_t value) {
  if (characteristic != tx_ || p->conn_handle != connectionId_.load()) return;
  const bool notifications = (value & 1) != 0;
  recordEvent({EventKind::Subscribe, p->conn_handle, notifications, 0, false});
  if (subscribed_.load() != notifications) {
    subscribed_.store(false);
    generation_.fetch_add(1);
    subscribed_.store(notifications);
  }
}
void BleTransport::onWrite(BLECharacteristic* characteristic, ble_gap_conn_desc* p) {
  if (characteristic != rx_ || p->conn_handle != connectionId_.load() || !ready()) return;
  const size_t size = characteristic->getLength();
  if (size <= wire::kFragmentHeader || size > wire::kMaxBleValue) {
    rxDrops_.fetch_add(1);
    return;
  }
  BleFragment fragment;
  fragment.generation = generation();
  fragment.receivedMs = millis();
  fragment.size = size;
  memcpy(fragment.bytes, characteristic->getData(), size);
  if (xQueueSend(ingress_, &fragment, 0) != pdTRUE) rxDrops_.fetch_add(1);
}
void BleTransport::onRead(BLECharacteristic* characteristic) {
  if (characteristic != info_) return;
  char info[200];
  snprintf(info, sizeof(info),
           "{\"protocol\":%u,\"boot_id\":%lu,\"generation\":%lu,\"max_payload\":1024,"
           "\"rx_drops\":%lu,\"pairing\":\"development_just_works\"}",
           static_cast<unsigned>(wire::kVersion), static_cast<unsigned long>(bootId_),
           static_cast<unsigned long>(generation()),
           static_cast<unsigned long>(rxDrops()));
  info_->setValue(reinterpret_cast<const uint8_t*>(info), strlen(info));
}
bool BleTransport::notify(const uint8_t* data, size_t size, uint32_t generation) {
  if (!current(generation) || size > valueCapacity()) return false;
  os_mbuf* buffer = ble_hs_mbuf_from_flat(data, size);
  if (!buffer) return false;
  // NimBLE consumes buffer on success AND failure. Direct return status avoids
  // mixing synchronous submission with asynchronous onStatus events.
  const int result = ble_gatts_notify_custom(connectionId_.load(), tx_->getHandle(), buffer);
  return result == 0 && current(generation);  // Submission only, not a phone ACK.
}
bool BleTransport::receive(BleFragment& fragment) {
  return xQueueReceive(ingress_, &fragment, 0) == pdTRUE;
}
void BleTransport::recordEvent(const Event& event) {
  xQueueSend(events_, &event, 0);
}
int BleTransport::onGapEvent(ble_gap_event* event, void* context) {
  if (event->type != BLE_GAP_EVENT_PHY_UPDATE_COMPLETE) return 0;
  auto& self = *static_cast<BleTransport*>(context);
  const auto& phy = event->phy_updated;
  if (phy.conn_handle == self.connectionId_.load()) {
    Event report{EventKind::Phy, phy.conn_handle, phy.status};
    if (phy.status == 0) {
      report.txPhy = phy.tx_phy;
      report.rxPhy = phy.rx_phy;
    }
    self.recordEvent(report);
  }
  return 0;
}

void BleTransport::requestPhy() {
  const uint16_t handle = connectionId_.load();
  if (handle == 0xffff || !encrypted_.load()) return;
  uint8_t tx = 0, rx = 0;
  const int readStatus = ble_gap_read_le_phy(handle, &tx, &rx);
  Serial.printf("BLE PHY initial status=%d TX=%s RX=%s\n",
                readStatus, phyName(tx), phyName(rx));
  if (config::kPreferCodedPhyS8) {
    const int status = ble_gap_set_prefered_le_phy(
        handle, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_S8);
    Serial.printf("BLE PHY request=Coded S=8 status=%d\n", status);
  }
}

void BleTransport::printEvents() {
  Event event;
  for (unsigned i = 0; i < 8 && xQueueReceive(events_, &event, 0) == pdTRUE; ++i) {
    const unsigned handle = event.handle;
    switch (event.kind) {
      case EventKind::Connect:
        Serial.printf("BLE connected handle=%u\n", handle);
        break;
      case EventKind::Disconnect:
        Serial.printf("BLE disconnected handle=%u\n", handle);
        break;
      case EventKind::Security:
        Serial.printf("BLE security handle=%u status=%d encrypted=%d key_size=%u\n",
                      handle, event.status, event.encrypted, static_cast<unsigned>(event.keySize));
        break;
      case EventKind::Subscribe:
        Serial.printf("BLE TX notify handle=%u enabled=%d\n", handle, event.status);
        break;
      case EventKind::Phy:
        Serial.printf("BLE PHY update handle=%u status=%d TX=%s RX=%s\n",
                      handle, event.status, phyName(event.txPhy), phyName(event.rxPhy));
        break;
    }
  }
}
void BleTransport::poll(uint32_t now) {
  printEvents();
  if (phyPending_.exchange(false)) requestPhy();
  if (restartAdvertising_.exchange(false)) restartAtMs_ = now + 150;
  if (restartAtMs_ && static_cast<int32_t>(now - restartAtMs_) >= 0) {
    restartAtMs_ = 0;
    if (!connected_.load()) BLEDevice::startAdvertising();
  }
}
