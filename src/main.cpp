#include <Arduino.h>
#include <BridgePayload.h>
#include <BridgeProtocol.h>
#include <esp_system.h>

#include "BleTransport.h"
#include "Config.h"

namespace {
HardwareSerial controller(1);
BleTransport ble;
wire::UartDecoder uartDecoder;
wire::Reassembler reassembler;

struct Pending {
  wire::Packet packet;
  uint32_t generation = 0;
  uint32_t receivedMs = 0;
};
// All of these queues belong exclusively to loop(). No callback accesses them.
wire::Queue<Pending, 1> stops;
wire::Queue<Pending, 8> commands;
wire::Queue<Pending, 8> responses;
wire::Queue<Pending, 4> telemetry;

struct Counters {
  uint32_t uartRx = 0, uartBad = 0, uartTx = 0;
  uint32_t bleRx = 0, bleBad = 0, bleTx = 0;
  uint32_t queueDrops = 0, expired = 0, notifyFailures = 0;
} stats;

struct UartTx {
  uint8_t bytes[wire::kMaxEncoded + 1]{};
  size_t size = 0, offset = 0;
  uint32_t generation = 0;
  bool fromPhone = false;
  bool needsDelimiter = false;
} uartTx;

struct BleTx {
  uint8_t raw[wire::kMaxRaw]{};
  size_t size = 0, offset = 0;
  uint16_t messageId = 0;
  uint32_t generation = 0, startedMs = 0, lastSendMs = 0;
  bool isTelemetry = false;
} bleTx;

uint32_t bootId, lastGeneration = 0, lastRxDrops = 0;
uint32_t assemblyStartedMs = 0, lastStatsMs = 0, linkSequence = 0;
bool wasReady = false, linkDirty = true;

bool isCommand(wire::Type type) {
  return type == wire::Type::Command || type == wire::Type::Stop;
}

void invalidateSession() {
  stops.clear();
  commands.clear();
  responses.clear();
  telemetry.clear();
  reassembler.reset();
  bleTx.size = 0;
  if (uartTx.size && uartTx.fromPhone) {
    // Finish an interrupted COBS frame with a delimiter before another packet.
    uartTx.needsDelimiter = uartTx.offset != 0;
    uartTx.size = uartTx.offset = 0;
  }
}

void readPhone(uint32_t now) {
  reassembler.expire(now, config::kBleAssemblyTimeoutMs);
  if (lastRxDrops != ble.rxDrops()) {
    lastRxDrops = ble.rxDrops();
    reassembler.reset();
  }
  BleFragment fragment;
  // Bound work per iteration so UART draining and STOP processing keep running.
  for (unsigned i = 0; i < 16 && ble.receive(fragment); ++i) {
    if (!ble.current(fragment.generation)) continue;
    if (static_cast<uint32_t>(millis() - fragment.receivedMs) > config::kCommandMaxAgeMs) {
      reassembler.reset();
      ++stats.expired;
      continue;
    }
    if (wire::read16(fragment.bytes + 4) == 0) assemblyStartedMs = fragment.receivedMs;
    reassembler.expire(fragment.receivedMs, config::kBleAssemblyTimeoutMs);
    Pending pending;
    const auto result = reassembler.feed(fragment.bytes, fragment.size, fragment.receivedMs, pending.packet);
    if (result == wire::FeedResult::Rejected) {
      ++stats.bleBad;
    } else if (result == wire::FeedResult::Complete) {
      JsonDocument json;
      if (!wire::allowedFromPhone(pending.packet.type) ||
          !payload::readJson(pending.packet, json) ||
          !payload::writeCbor(json.as<JsonVariantConst>(), pending.packet)) {
        ++stats.bleBad;
        continue;
      }
      pending.generation = fragment.generation;
      pending.receivedMs = assemblyStartedMs;
      ++stats.bleRx;
      if (pending.packet.type == wire::Type::Stop) {
        // A newer stop supersedes a queued stop. Never retain older queued starts.
        stops.clear();
        commands.clear();
        stops.push(pending);
      } else if (!commands.push(pending)) {
        ++stats.queueDrops;
      }
    }
  }
}

void readController(uint32_t now) {
  uartDecoder.expire(now, config::kUartFrameTimeoutMs);
  wire::Packet packet;
  for (unsigned i = 0; i < 2048 && controller.available(); ++i) {
    const auto result = uartDecoder.feed(static_cast<uint8_t>(controller.read()), now, packet);
    if (result == wire::FeedResult::Rejected) {
      ++stats.uartBad;
    } else if (result == wire::FeedResult::Complete) {
      if (!wire::allowedFromController(packet.type)) {
        ++stats.uartBad;
        continue;
      }
      ++stats.uartRx;
      if (!ble.ready()) continue;  // Reconnect must request a fresh snapshot.
      JsonDocument json;
      if (!payload::readCbor(packet, json) || !payload::writeJson(json.as<JsonVariantConst>(), packet)) {
        ++stats.uartBad;
        continue;
      }
      Pending pending;
      pending.packet = packet;
      pending.generation = ble.generation();
      pending.receivedMs = now;
      if (packet.type == wire::Type::Telemetry) {
        if (!telemetry.push(pending)) {
          Pending discarded;
          telemetry.pop(discarded);
          telemetry.push(pending);
          ++stats.queueDrops;
        }
      } else if (!responses.push(pending)) {
        // End-to-end ACK/retry belongs to phone and STM32, not to this bridge.
        ++stats.queueDrops;
      }
    }
  }
}

wire::Packet makeLinkState() {
  wire::Packet packet;
  packet.type = wire::Type::LinkState;
  packet.sequence = ++linkSequence;
  JsonDocument state;
  state["boot_id"] = bootId;
  state["generation"] = ble.generation();
  state["ready"] = ble.ready();
  if (!state.overflowed()) payload::writeCbor(state.as<JsonVariantConst>(), packet);
  return packet;
}

void writeController(uint32_t now) {
  if (uartTx.size && uartTx.fromPhone && !ble.current(uartTx.generation)) {
    uartTx.needsDelimiter = uartTx.offset != 0;
    uartTx.size = uartTx.offset = 0;
  }
  if (uartTx.needsDelimiter) {
    if (controller.availableForWrite() < 1) return;
    controller.write(static_cast<uint8_t>(0));
    uartTx.needsDelimiter = false;
  }
  if (!uartTx.size) {
    Pending pending;
    bool found = stops.pop(pending);
    bool fromPhone = found;
    if (!found && linkDirty) {
      pending.packet = makeLinkState();
      if (!pending.packet.size) return;
      linkDirty = false;
      found = true;
    } else if (!found) {
      found = commands.pop(pending);
      fromPhone = found;
    }
    if (!found) return;
    if (fromPhone && (!ble.current(pending.generation) ||
        (isCommand(pending.packet.type) && static_cast<uint32_t>(now - pending.receivedMs) > config::kCommandMaxAgeMs))) {
      ++stats.expired;
      return;
    }
    uint8_t raw[wire::kMaxRaw];
    const size_t rawSize = wire::encodeRaw(pending.packet, raw, sizeof(raw), wire::kUartVersion);
    uartTx.size = wire::cobsEncode(raw, rawSize, uartTx.bytes, sizeof(uartTx.bytes) - 1);
    if (!rawSize || !uartTx.size) {
      uartTx.size = 0;
      return;
    }
    uartTx.bytes[uartTx.size++] = 0;
    uartTx.offset = 0;
    uartTx.generation = pending.generation;
    uartTx.fromPhone = fromPhone;
  }
  const int available = controller.availableForWrite();
  if (available <= 0) return;
  size_t count = uartTx.size - uartTx.offset;
  if (count > static_cast<size_t>(available)) count = available;
  if (count > 64) count = 64;
  uartTx.offset += controller.write(uartTx.bytes + uartTx.offset, count);
  if (uartTx.offset == uartTx.size) {
    ++stats.uartTx;
    uartTx.size = uartTx.offset = 0;
  }
}

void writePhone(uint32_t now) {
  if (!ble.ready()) return;
  // A response may replace an unfinished telemetry message (new offset 0).
  if (bleTx.size && bleTx.isTelemetry && responses.size()) bleTx.size = 0;
  if (bleTx.size && (static_cast<uint32_t>(now - bleTx.startedMs) > config::kBleSendTimeoutMs ||
                    !ble.current(bleTx.generation))) {
    bleTx.size = 0;
    ++stats.expired;
  }
  if (!bleTx.size) {
    Pending pending;
    bool found = responses.pop(pending);
    bleTx.isTelemetry = !found;
    if (!found) found = telemetry.pop(pending);
    if (!found) return;
    if (!ble.current(pending.generation) ||
        static_cast<uint32_t>(now - pending.receivedMs) > config::kBleSendTimeoutMs) {
      ++stats.expired;
      return;
    }
    bleTx.size = wire::encodeRaw(pending.packet, bleTx.raw, sizeof(bleTx.raw));
    bleTx.offset = 0;
    ++bleTx.messageId;
    bleTx.generation = pending.generation;
    bleTx.startedMs = now;
  }
  if (static_cast<uint32_t>(now - bleTx.lastSendMs) < config::kNotifyIntervalMs) return;
  bleTx.lastSendMs = now;
  uint8_t fragment[wire::kMaxBleValue];
  size_t count = bleTx.size - bleTx.offset;
  const size_t capacity = ble.valueCapacity() - wire::kFragmentHeader;
  if (count > capacity) count = capacity;
  wire::write16(fragment, bleTx.messageId);
  wire::write16(fragment + 2, bleTx.size);
  wire::write16(fragment + 4, bleTx.offset);
  memcpy(fragment + wire::kFragmentHeader, bleTx.raw + bleTx.offset, count);
  if (!ble.notify(fragment, wire::kFragmentHeader + count, bleTx.generation)) {
    // Abandon on submission failure. Do not replay an ambiguous fragment.
    bleTx.size = 0;
    ++stats.notifyFailures;
    return;
  }
  bleTx.offset += count;
  if (bleTx.offset == bleTx.size) {
    bleTx.size = 0;
    ++stats.bleTx;
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  controller.setRxBufferSize(4096);
  controller.begin(config::kUartBaud, SERIAL_8N1, config::kUartRxPin, config::kUartTxPin);
  // Discard any partial frame left in the STM32 parser after an ESP32 reboot.
  controller.write(static_cast<uint8_t>(0));
  bootId = esp_random();
  ble.begin(bootId);
  Serial.printf("UART1: RX=%d TX=%d baud=%lu; UART protocol=%u; boot=%lu\n",
                config::kUartRxPin, config::kUartTxPin,
                static_cast<unsigned long>(config::kUartBaud), static_cast<unsigned>(wire::kUartVersion),
                static_cast<unsigned long>(bootId));
}

void loop() {
  const uint32_t now = millis();
  ble.poll(now);
  const uint32_t generation = ble.generation();
  const bool ready = ble.ready();
  if (generation != lastGeneration || ready != wasReady) {
    invalidateSession();
    lastGeneration = generation;
    wasReady = ready;
    linkDirty = true;
    Serial.printf("BLE generation=%lu ready=%d\n", static_cast<unsigned long>(generation), ready);
  }
  readPhone(now);
  writeController(millis());
  readController(millis());
  writePhone(millis());
  if (static_cast<uint32_t>(now - lastStatsMs) >= config::kStatsIntervalMs) {
    lastStatsMs = now;
    Serial.printf("UART rx=%lu bad=%lu tx=%lu | BLE rx=%lu bad=%lu tx=%lu | "
                  "queue_drop=%lu ingress_drop=%lu expired=%lu notify_fail=%lu heap=%lu\n",
                  static_cast<unsigned long>(stats.uartRx), static_cast<unsigned long>(stats.uartBad),
                  static_cast<unsigned long>(stats.uartTx), static_cast<unsigned long>(stats.bleRx),
                  static_cast<unsigned long>(stats.bleBad), static_cast<unsigned long>(stats.bleTx),
                  static_cast<unsigned long>(stats.queueDrops), static_cast<unsigned long>(ble.rxDrops()),
                  static_cast<unsigned long>(stats.expired), static_cast<unsigned long>(stats.notifyFailures),
                  static_cast<unsigned long>(ESP.getFreeHeap()));
  }
  delay(1);
}
