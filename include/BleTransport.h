#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BridgeProtocol.h>
#include <atomic>
#include <freertos/queue.h>

// Callback task only copies fragments into a bounded FreeRTOS queue.
// Protocol decoding and all UART I/O run in the Arduino loop task.
struct BleFragment {
  uint32_t generation = 0;
  uint32_t receivedMs = 0;
  uint16_t size = 0;
  uint8_t bytes[wire::kMaxBleValue]{};
};

class BleTransport : private BLEServerCallbacks,
                     private BLECharacteristicCallbacks,
                     private BLESecurityCallbacks {
 public:
  void begin(uint32_t bootId);
  void poll(uint32_t now);
  bool receive(BleFragment& fragment);
  bool notify(const uint8_t* data, size_t size, uint32_t generation);
  bool ready() const { return connected_.load() && encrypted_.load() && subscribed_.load(); }
  bool current(uint32_t generation) const { return generation_.load() == generation && ready(); }
  uint32_t generation() const { return generation_.load(); }
  uint32_t rxDrops() const { return rxDrops_.load(); }
  uint16_t valueCapacity() const { return mtu_.load() - 3; }

 private:
  using BLEServerCallbacks::onConnect;
  using BLEServerCallbacks::onDisconnect;
  using BLECharacteristicCallbacks::onWrite;
  using BLECharacteristicCallbacks::onRead;
  using BLESecurityCallbacks::onAuthenticationComplete;
  void onConnect(BLEServer*, ble_gap_conn_desc*) override;
  void onDisconnect(BLEServer*, ble_gap_conn_desc*) override;
  void onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t) override;
  void onWrite(BLECharacteristic*, ble_gap_conn_desc*) override;
  void onRead(BLECharacteristic*) override;
  void onSubscribe(BLECharacteristic*, ble_gap_conn_desc*, uint16_t) override;
  bool onSecurityRequest() override { return connected_.load(); }
  void onAuthenticationComplete(ble_gap_conn_desc*, int) override;

  QueueHandle_t ingress_ = nullptr;
  BLEServer* server_ = nullptr;
  BLECharacteristic* rx_ = nullptr;
  BLECharacteristic* tx_ = nullptr;
  BLECharacteristic* info_ = nullptr;
  std::atomic<bool> connected_{false};
  std::atomic<bool> encrypted_{false};
  std::atomic<bool> subscribed_{false};
  std::atomic<bool> restartAdvertising_{false};
  std::atomic<uint16_t> mtu_{23};
  std::atomic<uint16_t> connectionId_{0xffff};
  std::atomic<uint32_t> generation_{0};
  std::atomic<uint32_t> rxDrops_{0};
  uint32_t bootId_ = 0;
  uint32_t restartAtMs_ = 0;
};
