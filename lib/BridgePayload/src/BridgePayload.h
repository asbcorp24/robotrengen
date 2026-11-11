#pragma once

#include <ArduinoJson.h>
#include <BridgeProtocol.h>

namespace payload {
constexpr uint8_t kMaxNesting = 8;

bool readCbor(const wire::Packet& packet, JsonDocument& doc);
bool writeCbor(JsonVariantConst value, wire::Packet& packet);
bool readJson(const wire::Packet& packet, JsonDocument& doc);
bool writeJson(JsonVariantConst value, wire::Packet& packet);
}  // namespace payload
