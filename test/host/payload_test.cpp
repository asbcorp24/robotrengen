#include <BridgePayload.h>
#include <cassert>
#include <cstring>
#include <cstdio>

int main() {
  wire::Packet packet;
  const char request[] = "{\"op\":\"start\"}";
  memcpy(packet.payload, request, sizeof(request) - 1);
  packet.size = sizeof(request) - 1;
  JsonDocument doc;
  assert(payload::readJson(packet, doc));
  assert(payload::writeCbor(doc.as<JsonVariantConst>(), packet));
  const uint8_t expected[] = {0xa1, 0x62, 'o', 'p', 0x65, 's', 't', 'a', 'r', 't'};
  assert(packet.size == sizeof(expected) && !memcmp(packet.payload, expected, sizeof(expected)));
  assert(payload::readCbor(packet, doc));
  assert(payload::writeJson(doc.as<JsonVariantConst>(), packet));
  assert(packet.size == sizeof(request) - 1 && !memcmp(packet.payload, request, packet.size));

  doc.clear();
  doc["boot_id"] = UINT32_MAX;
  JsonArray values = doc["values"].to<JsonArray>();
  values.add(-42);
  values.add(1.25);
  values.add(true);
  values.add(nullptr);
  values.add(u8"Ошибка");
  assert(payload::writeCbor(doc.as<JsonVariantConst>(), packet));
  assert(payload::readCbor(packet, doc));
  assert(doc["boot_id"].as<uint32_t>() == UINT32_MAX);
  assert(doc["values"][0].as<int>() == -42);
  assert(doc["values"][1].as<double>() == 1.25);
  assert(doc["values"][2].as<bool>());
  assert(doc["values"][3].isNull());
  assert(doc["values"][4].as<JsonString>() == u8"Ошибка");
  --packet.size;
  assert(!payload::readCbor(packet, doc));

  const uint8_t duplicate[] = {0xa2, 0x61, 'x', 0xf6, 0x61, 'x', 0x01};
  memcpy(packet.payload, duplicate, sizeof(duplicate));
  packet.size = sizeof(duplicate);
  assert(!payload::readCbor(packet, doc));

  const uint8_t half[] = {0xa1, 0x61, 'x', 0xf9, 0x3e, 0x00};
  memcpy(packet.payload, half, sizeof(half));
  packet.size = sizeof(half);
  assert(payload::readCbor(packet, doc));
  assert(doc["x"].as<double>() == 1.5);

  char large[wire::kMaxPayload + 1];
  memset(large, 'x', sizeof(large) - 1);
  large[sizeof(large) - 1] = 0;
  doc["oversize"] = large;
  assert(!payload::writeJson(doc.as<JsonVariantConst>(), packet));
  assert(!payload::writeCbor(doc.as<JsonVariantConst>(), packet));
  puts("PASS: JSON/CBOR conversion, nested values, UTF-8, malformed and oversized payloads");
}
