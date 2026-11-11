#include <Arduino.h>
#include <BridgeProtocol.h>
#include <Serial.h>
#include <string.h>

namespace {
constexpr uint32_t kBootId = 1;  // Fixed identifier for the bench simulator.
constexpr uint32_t kSchemaRevision = 1;
constexpr uint32_t kBatteryVoltage = 1001;
constexpr uint32_t kBatteryTemperature = 1002;
constexpr uint32_t kDistance = 1101;
constexpr uint32_t kMoving = 1301;

Uart bridge(PA_10, PA_9);  // USART1: RX, TX.
wire::UartDecoder decoder;
wire::Packet incoming, outgoing;
uint8_t raw[wire::kMaxRaw];
uint8_t frame[wire::kMaxEncoded + 1];
uint32_t sequence = 0, errorRevision = 0, distanceMm = 0;
uint32_t lastTick = 0, lastTelemetry = 0, lastErrors = 0;
bool moving = false, fault = false;

// Only the CBOR types emitted by this simulator are needed here.
class Cbor {
 public:
  explicit Cbor(wire::Packet& packet) : packet_(packet) { packet_.size = 0; }
  void number(uint32_t value) { item(0, value); }
  void map(uint8_t count) { item(5, count); }
  void array(uint8_t count) { item(4, count); }
  void boolean(bool value) { byte(value ? 0xf5 : 0xf4); }
  void string(const char* value) {
    item(3, strlen(value));
    while (*value) byte(static_cast<uint8_t>(*value++));
  }
  bool valid() const { return valid_; }

 private:
  void byte(uint8_t value) {
    if (packet_.size < wire::kMaxPayload) packet_.payload[packet_.size++] = value;
    else valid_ = false;
  }
  void item(uint8_t major, uint32_t value) {
    const uint8_t prefix = major << 5;
    if (value < 24) {
      byte(prefix | value);
      return;
    }
    const int bytes = value <= 0xff ? 1 : (value <= 0xffff ? 2 : 4);
    byte(prefix | (bytes == 1 ? 24 : (bytes == 2 ? 25 : 26)));
    for (int shift = (bytes - 1) * 8; shift >= 0; shift -= 8) byte(value >> shift);
  }

  wire::Packet& packet_;
  bool valid_ = true;
};

void send(const Cbor& payload, wire::Type type, uint32_t seq) {
  if (!payload.valid()) return;
  outgoing.type = type;
  outgoing.sequence = seq;
  const size_t rawSize = wire::encodeRaw(outgoing, raw, sizeof(raw));
  if (!rawSize) return;
  const size_t encodedSize = wire::cobsEncode(raw, rawSize, frame, sizeof(frame) - 1);
  if (!encodedSize) return;
  frame[encodedSize] = 0;
  bridge.write(frame, encodedSize + 1);
}

void sendHello() {
  Cbor out(outgoing);
  out.map(3);
  out.number(0); out.string("bluepill-simulator");
  out.number(1); out.number(kBootId);
  out.number(2); out.number(kSchemaRevision);
  send(out, wire::Type::Hello, ++sequence);
}

void describe(Cbor& out, uint32_t id, const char* key, const char* label,
              const char* type, const char* unit) {
  out.map(7);
  out.string("id"); out.number(id);
  out.string("key"); out.string(key);
  out.string("name"); out.string(label);
  out.string("type"); out.string(type);
  out.string("unit"); out.string(unit);
  out.string("access"); out.string("read");
  out.string("period_ms"); out.number(200);
}

void sendSchema() {
  Cbor out(outgoing);
  out.map(4);
  out.number(0); out.number(kSchemaRevision);
  out.number(1); out.number(0);
  out.number(2); out.number(1);
  out.number(3); out.array(4);
  describe(out, kBatteryVoltage, "drive_battery_voltage", "Battery voltage", "uint32", "mV");
  describe(out, kBatteryTemperature, "drive_battery_temperature", "Battery temperature", "int32", "degC");
  describe(out, kDistance, "distance", "Simulated distance", "uint32", "mm");
  describe(out, kMoving, "moving", "Simulated motion", "bool", "");
  send(out, wire::Type::Schema, ++sequence);
}

void sendTelemetry(uint32_t now) {
  Cbor out(outgoing);
  out.map(3);
  out.number(0); out.number(kBootId);
  out.number(1); out.number(now);
  out.number(2); out.map(4);
  out.number(kBatteryVoltage); out.number(24700 - (now / 1000) % 500);
  out.number(kBatteryTemperature); out.number(35 + (now / 5000) % 5);
  out.number(kDistance); out.number(distanceMm);
  out.number(kMoving); out.boolean(moving);
  send(out, wire::Type::Telemetry, ++sequence);
}

void sendErrors() {
  Cbor out(outgoing);
  out.map(3);
  out.number(0); out.number(kBootId);
  out.number(1); out.number(errorRevision);
  out.number(2); out.array(fault ? 1 : 0);
  if (fault) {
    out.map(3);
    out.string("source"); out.string("simulator");
    out.string("code"); out.number(1);
    out.string("severity"); out.string("fault");
  }
  send(out, wire::Type::ErrorsSnapshot, ++sequence);
}

void sendResult(uint32_t request, const char* reason) {
  Cbor out(outgoing);
  out.map(reason ? 2 : 1);
  out.string("status"); out.string(reason ? "rejected" : "completed");
  if (reason) {
    out.string("reason"); out.string(reason);
  }
  send(out, wire::Type::CommandResult, request);
}

void setFault(bool enabled) {
  if (fault == enabled) return;
  fault = enabled;
  if (fault) moving = false;
  ++errorRevision;
  Cbor out(outgoing);
  out.map(6);
  out.string("boot_id"); out.number(kBootId);
  out.string("event_id"); out.number(errorRevision);
  out.string("source"); out.string("simulator");
  out.string("code"); out.number(1);
  out.string("transition"); out.string(fault ? "raised" : "cleared");
  out.string("time_ms"); out.number(millis());
  send(out, wire::Type::ErrorEvent, ++sequence);
}

const char* simulateCommand(const wire::Packet& packet) {
  // Exact CBOR map {0: operation}, with a small integer encoded in one byte.
  if (packet.size != 3 || packet.payload[0] != 0xa1 || packet.payload[1] != 0) {
    return "invalid_simulator_command";
  }
  switch (packet.payload[2]) {
    case 1:
      if (fault) return "simulated_fault_active";
      moving = true;
      return nullptr;
    case 2:
      setFault(true);
      return nullptr;
    case 3:
      setFault(false);
      return nullptr;
    default:
      return "unknown_simulator_operation";
  }
}

void handle(const wire::Packet& packet) {
  switch (packet.type) {
    case wire::Type::Hello:
      sendHello();
      break;
    case wire::Type::SchemaRequest:
      sendSchema();
      break;
    case wire::Type::SnapshotRequest:
      sendTelemetry(millis());
      sendErrors();
      break;
    case wire::Type::Command:
      sendResult(packet.sequence, simulateCommand(packet));
      sendTelemetry(millis());
      sendErrors();
      break;
    case wire::Type::Stop:
      moving = false;
      sendResult(packet.sequence, nullptr);
      sendTelemetry(millis());
      break;
    default:
      break;
  }
}
}  // namespace

void setup() {
  bridge.begin(115200, SERIAL_8N1);
  bridge.write(static_cast<uint8_t>(0));
  lastTick = millis();
}

void loop() {
  const uint32_t tick = millis();
  if (moving) distanceMm += tick - lastTick;  // Model speed of 1000 mm/s.
  lastTick = tick;
  decoder.expire(tick, 250);
  for (unsigned n = 0; n < 512 && bridge.available(); ++n) {
    const auto byte = static_cast<uint8_t>(bridge.read());
    if (decoder.feed(byte, millis(), incoming) == wire::FeedResult::Complete) {
      handle(incoming);
    }
  }
  const uint32_t now = millis();
  if (now - lastTelemetry >= 200) {
    lastTelemetry = now;
    sendTelemetry(now);
  }
  if (now - lastErrors >= 1000) {
    lastErrors = now;
    sendErrors();
  }
}
