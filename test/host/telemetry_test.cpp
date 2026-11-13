#include <cassert>
#include <cstdio>
#include <map>
#include <set>
#include <string>

#include "../../src/bluepill/main.cpp"

uint32_t testMillis = 1234;

template <typename Callback>
void readFrames(Callback callback) {
  wire::UartDecoder reader;
  wire::Packet packet;
  for (uint8_t byte : bridge.bytes) {
    const auto result = reader.feed(byte, testMillis, packet);
    assert(result != wire::FeedResult::Rejected);
    if (result != wire::FeedResult::Complete) continue;
    JsonDocument doc;
    assert(payload::readCbor(packet, doc));
    assert(measureJson(doc) <= wire::kMaxPayload);
    callback(packet.type, doc);
  }
  bridge.bytes.clear();
}

int main() {
  std::map<std::string, JsonDocument> groups;
  sendTelemetry(testMillis);
  readFrames([&](wire::Type type, const JsonDocument& doc) {
    assert(type == wire::Type::Telemetry);
    assert(doc["source"] == "stm32" && doc["simulated"] == true);
    groups[doc["group"].as<const char*>()] = doc;
  });
  assert(groups.size() == 4);
  std::set<std::string> keys;
  for (const auto& group : groups) {
    for (JsonPairConst pair : group.second["values"].as<JsonObjectConst>()) keys.insert(pair.key().c_str());
  }
  assert(keys.size() == sizeof(catalog::kStm32Fields) / sizeof(catalog::Field));
  for (const auto& field : catalog::kStm32Fields) assert(keys.count(field.key));
  assert(groups["motion"]["values"]["brake_force_pct"].isNull());
  assert(groups["battery"]["values"]["drive_battery_voltage"].as<double>() <= 200);
  assert(groups["diagnostics"]["values"]["magnetic_signal_pct"].size() == 2);

  std::set<std::string> described;
  sendSchema();
  const size_t pages = catalog::pageCount(catalog::kStm32Fields);
  for (size_t i = 0; i < pages; ++i) {
    sendSchemaPage(testMillis);
    unsigned received = 0;
    readFrames([&](wire::Type type, const JsonDocument& doc) {
      ++received;
      assert(type == wire::Type::Schema && doc["part_index"].as<size_t>() == i);
      for (JsonObjectConst field : doc["fields"].as<JsonArrayConst>())
        described.insert(field["key"].as<const char*>());
    });
    assert(received == 1);
    testMillis += 700;
  }
  assert(described == keys && !schemaRequested);
  for (size_t page = 0; page < catalog::pageCount(catalog::kEsp32Fields); ++page) {
    JsonDocument doc;
    catalog::schemaPage(doc, "esp32", catalog::kEsp32Fields, page);
    assert(measureJson(doc) <= wire::kMaxPayload);
  }

  setFault(true);
  sendFastTelemetry(testMillis);
  sendErrors();
  readFrames([&](wire::Type type, const JsonDocument& doc) {
    if (type == wire::Type::ErrorsSnapshot) assert(doc["active"][0]["code"] == 1);
    if (type == wire::Type::Telemetry && doc["group"] == "xray")
      assert(doc["values"]["xray_error_mask"] == 1);
  });
  puts("PASS: all STM32 telemetry fields, schema pages fit CBOR/JSON, fault mask and null validity");
}
