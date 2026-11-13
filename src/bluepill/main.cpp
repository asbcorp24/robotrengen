#include <Arduino.h>
#include <BridgePayload.h>
#include <BridgeProtocol.h>
#include <Serial.h>
#include <TelemetryCatalog.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint32_t kBootId = 1;  // Fixed identifier for the bench simulator.
Uart bridge(PA_10, PA_9);  // USART1: RX, TX.
wire::UartDecoder decoder;
wire::Packet incoming, outgoing;
uint8_t raw[wire::kMaxRaw];
uint8_t frame[wire::kMaxEncoded + 1];
uint32_t sequence = 0, errorRevision = 0, distanceMm = 0;
uint64_t travelMm = 0;  // Volatile model; hardware odometer will require persistent storage.
uint32_t lastSlowTelemetry = 0, lastSchema = 0;
size_t schemaPage = 0;
bool schemaRequested = false;
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
  out["source"] = "stm32";
  out["simulated"] = true;
  out["device_id"] = "bluepill-simulator";
  out["boot_id"] = kBootId;
  out["schema_revision"] = catalog::kRevision;
  send(out, wire::Type::Hello, ++sequence);
}

void sendSchema() {
  schemaPage = 0;
  schemaRequested = true;
  lastSchema = millis() - 700;
}

void sendSchemaPage(uint32_t now) {
  if (!schemaRequested || now - lastSchema < 700) return;
  JsonDocument out;
  catalog::schemaPage(out, "stm32", catalog::kStm32Fields, schemaPage);
  send(out, wire::Type::Schema, ++sequence);
  lastSchema = now;
  if (++schemaPage == catalog::pageCount(catalog::kStm32Fields)) schemaRequested = false;
}

JsonObject telemetryGroup(JsonDocument& out, const char* group, uint32_t now) {
  out["source"] = "stm32";
  out["simulated"] = true;
  out["boot_id"] = kBootId;
  out["sample_ms"] = now;
  out["group"] = group;
  return out["values"].to<JsonObject>();
}

void sendFastTelemetry(uint32_t now) {
  JsonDocument out;
  JsonObject values = telemetryGroup(out, "motion", now);
  values["distance"] = (distanceMm / 100) / 10.0;  // 0.1 m reporting step.
  values["moving"] = moving;
  values["crawler_state"] = moving ? "moving" : "stop";
  values["sensor_start_triggered"] = false;
  values["sensor_end_triggered"] = false;
  values["sensor_water_triggered"] = false;
  values["sensor_obstacle_triggered"] = false;
  values["roll"] = round(sin(now / 5000.0) * 25) / 10.0;
  values["pitch"] = round(cos(now / 6000.0) * 15) / 10.0;
  values["brake_command_pct"] = moving ? 0 : 65;
  // No force sensor on the bench; report unavailable rather than a PWM value.
  values["brake_force_pct"] = nullptr;
  send(out, wire::Type::Telemetry, ++sequence);

  out.clear();
  values = telemetryGroup(out, "xray", now);
  values["xray_state"] = fault ? "errors" : "ready";
  values["xray_voltage"] = 0;
  values["xray_current"] = 0;
  values["exposure_remaining_s"] = 0;
  values["exposure_delay_remaining_s"] = 0;
  values["xray_error_mask"] = fault ? 1 : 0;  // Bit 0 represents error code 1.
  send(out, wire::Type::Telemetry, ++sequence);
}

void sendSlowTelemetry(uint32_t now) {
  JsonDocument out;
  JsonObject values = telemetryGroup(out, "battery", now);
  values["odometer_km"] = 100 + static_cast<uint32_t>(travelMm / 1000000);
  values["drive_battery_voltage"] = (247 - (now / 10000) % 5) / 10.0;
  values["drive_battery_current"] = moving ? 8.5 : 0.4;  // Positive = discharge.
  values["drive_battery_temperature"] = 35 + (now / 5000) % 5;
  values["drive_battery_soc"] = 85;
  values["xray_battery_voltage"] = 48.2;
  values["xray_battery_current"] = 0.2;
  values["xray_battery_temperature"] = 32;
  values["xray_battery_soc"] = 90;
  send(out, wire::Type::Telemetry, ++sequence);

  out.clear();
  values = telemetryGroup(out, "diagnostics", now);
  JsonArray magnetic = values["magnetic_signal_pct"].to<JsonArray>();
  magnetic.add(42 + (now / 1000) % 10);
  magnetic.add(50 + (now / 1500) % 10);  // Two simulated receivers.
  values["imu_health"] = "ok";
  values["tof_health"] = "ok";
  values["analog_health"] = "ok";
  values["sensor_start_health"] = "ok";
  values["sensor_end_health"] = "ok";
  values["sensor_water_health"] = "ok";
  values["sensor_obstacle_health"] = "ok";
  send(out, wire::Type::Telemetry, ++sequence);
}

void sendTelemetry(uint32_t now) {
  sendFastTelemetry(now);
  sendSlowTelemetry(now);
}

void sendErrors() {
  JsonDocument out;
  out["boot_id"] = kBootId;
  out["source"] = "stm32";
  out["simulated"] = true;
  out["revision"] = errorRevision;
  JsonArray active = out["active"].to<JsonArray>();
  if (fault) {
    JsonObject error = active.add<JsonObject>();
    error["source"] = "xray";
    error["code"] = 1;
    error["severity"] = "fault";
  }
  send(out, wire::Type::ErrorsSnapshot, ++sequence);
}

void sendResult(uint32_t request, const char* reason) {
  JsonDocument out;
  out["source"] = "stm32";
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
  out["source"] = "stm32";
  out["subsystem"] = "xray";
  out["simulated"] = true;
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
  if (moving) {
    const uint32_t step = tick - lastTick < 10000000 - distanceMm
                              ? tick - lastTick : 10000000 - distanceMm;
    distanceMm += step;  // Model speed 1000 mm/s, limit 10000 m.
    travelMm += step;
    if (distanceMm == 10000000) moving = false;
  }
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
    sendFastTelemetry(now);
  }
  if (now - lastSlowTelemetry >= 1000) {
    lastSlowTelemetry = now;
    sendSlowTelemetry(now);
  }
  sendSchemaPage(now);
  if (now - lastErrors >= 1000) {
    lastErrors = now;
    sendErrors();
  }
}
