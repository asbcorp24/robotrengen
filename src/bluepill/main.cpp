#include <Arduino.h>
#include <BridgePayload.h>
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

void send(const JsonDocument& doc, wire::Type type, uint32_t seq) {
  if (doc.overflowed() || !payload::writeCbor(doc.as<JsonVariantConst>(), outgoing)) return;
  outgoing.type = type;
  outgoing.sequence = seq;
  const size_t rawSize = wire::encodeRaw(outgoing, raw, sizeof(raw), wire::kUartVersion);
  if (!rawSize) return;
  const size_t encodedSize = wire::cobsEncode(raw, rawSize, frame, sizeof(frame) - 1);
  if (!encodedSize) return;
  frame[encodedSize] = 0;
  bridge.write(frame, encodedSize + 1);
}

void sendHello() {
  JsonDocument out;
  out["device_id"] = "bluepill-simulator";
  out["boot_id"] = kBootId;
  out["schema_revision"] = kSchemaRevision;
  send(out, wire::Type::Hello, ++sequence);
}

void describe(JsonArray fields, uint32_t id, const char* key, const char* label,
              const char* type, const char* unit) {
  JsonObject field = fields.add<JsonObject>();
  field["id"] = id;
  field["key"] = key;
  field["name"] = label;
  field["type"] = type;
  field["unit"] = unit;
  field["access"] = "read";
  field["period_ms"] = 200;
}

void sendSchema() {
  JsonDocument out;
  out["revision"] = kSchemaRevision;
  out["part_index"] = 0;
  out["parts_total"] = 1;
  JsonArray fields = out["fields"].to<JsonArray>();
  describe(fields, kBatteryVoltage, "drive_battery_voltage", "Battery voltage", "uint32", "mV");
  describe(fields, kBatteryTemperature, "drive_battery_temperature", "Battery temperature", "int32", "degC");
  describe(fields, kDistance, "distance", "Simulated distance", "uint32", "mm");
  describe(fields, kMoving, "moving", "Simulated motion", "bool", "");
  send(out, wire::Type::Schema, ++sequence);
}

void sendTelemetry(uint32_t now) {
  JsonDocument out;
  out["boot_id"] = kBootId;
  out["sample_ms"] = now;
  JsonObject values = out["values"].to<JsonObject>();
  values["drive_battery_voltage"] = 24700 - (now / 1000) % 500;
  values["drive_battery_temperature"] = 35 + (now / 5000) % 5;
  values["distance"] = distanceMm;
  values["moving"] = moving;
  send(out, wire::Type::Telemetry, ++sequence);
}

void sendErrors() {
  JsonDocument out;
  out["boot_id"] = kBootId;
  out["revision"] = errorRevision;
  JsonArray active = out["active"].to<JsonArray>();
  if (fault) {
    JsonObject error = active.add<JsonObject>();
    error["source"] = "simulator";
    error["code"] = 1;
    error["severity"] = "fault";
  }
  send(out, wire::Type::ErrorsSnapshot, ++sequence);
}

void sendResult(uint32_t request, const char* reason) {
  JsonDocument out;
  out["status"] = reason ? "rejected" : "completed";
  if (reason) out["reason"] = reason;
  send(out, wire::Type::CommandResult, request);
}

void setFault(bool enabled) {
  if (fault == enabled) return;
  fault = enabled;
  if (fault) moving = false;
  ++errorRevision;
  JsonDocument out;
  out["boot_id"] = kBootId;
  out["event_id"] = errorRevision;
  out["source"] = "simulator";
  out["code"] = 1;
  out["transition"] = fault ? "raised" : "cleared";
  out["time_ms"] = millis();
  send(out, wire::Type::ErrorEvent, ++sequence);
}

const char* simulateCommand(const wire::Packet& packet) {
  JsonDocument command;
  if (!payload::readCbor(packet, command)) return "invalid_simulator_command";
  const JsonString op = command["op"].as<JsonString>();
  if (op.isNull()) {
    return "invalid_simulator_command";
  }
  if (op == "start") {
    if (fault) return "simulated_fault_active";
    moving = true;
  } else if (op == "raise_fault") {
    setFault(true);
  } else if (op == "clear_fault") {
    setFault(false);
  } else {
    return "unknown_simulator_operation";
  }
  return nullptr;
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
