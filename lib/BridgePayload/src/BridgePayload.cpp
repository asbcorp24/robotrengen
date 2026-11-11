#include "BridgePayload.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace payload {
namespace {
class CborWriter {
 public:
  explicit CborWriter(uint8_t* bytes) : bytes_(bytes) {}
  size_t size() const { return size_; }

  bool write(JsonVariantConst value, uint8_t depth) {
    if (!depth) return false;
    if (value.isNull()) return byte(0xf6);
    if (value.is<bool>()) return byte(value.as<bool>() ? 0xf5 : 0xf4);
    if (value.is<uint64_t>()) return head(0, value.as<uint64_t>());
    if (value.is<int64_t>()) return head(1, -(value.as<int64_t>() + 1));
    if (value.is<double>()) {
      const double number = value.as<double>();
      uint64_t bits = 0;
      static_assert(sizeof(number) == sizeof(bits), "CBOR requires binary64");
      memcpy(&bits, &number, sizeof(bits));
      return std::isfinite(number) && byte(0xfb) && bigEndian(bits, 8);
    }
    if (value.is<JsonString>()) return string(value.as<JsonString>());
    if (value.is<JsonArrayConst>()) {
      const auto array = value.as<JsonArrayConst>();
      if (!head(4, array.size())) return false;
      for (const auto element : array) if (!write(element, depth - 1)) return false;
      return true;
    }
    if (value.is<JsonObjectConst>()) {
      const auto object = value.as<JsonObjectConst>();
      if (!head(5, object.size())) return false;
      for (const auto pair : object) {
        if (!string(pair.key()) || !write(pair.value(), depth - 1)) return false;
      }
      return true;
    }
    return false;
  }

 private:
  bool byte(uint8_t value) {
    if (size_ == wire::kMaxPayload) return false;
    bytes_[size_++] = value;
    return true;
  }

  bool bigEndian(uint64_t value, unsigned count) {
    for (unsigned i = count; i; --i) {
      if (!byte(value >> (8 * (i - 1)))) return false;
    }
    return true;
  }

  bool head(uint8_t major, uint64_t value) {
    const uint8_t prefix = major << 5;
    if (value < 24) return byte(prefix | value);
    if (value <= UINT8_MAX) return byte(prefix | 24) && bigEndian(value, 1);
    if (value <= UINT16_MAX) return byte(prefix | 25) && bigEndian(value, 2);
    if (value <= UINT32_MAX) return byte(prefix | 26) && bigEndian(value, 4);
    return byte(prefix | 27) && bigEndian(value, 8);
  }

  bool string(JsonString value) {
    if (!head(3, value.size()) || value.size() > wire::kMaxPayload - size_) return false;
    memcpy(bytes_ + size_, value.c_str(), value.size());
    size_ += value.size();
    return true;
  }

  uint8_t* bytes_;
  size_t size_ = 0;
};

class CborReader {
 public:
  CborReader(const uint8_t* bytes, size_t size) : cursor_(bytes), end_(bytes + size) {}
  bool done() const { return cursor_ == end_; }

  bool read(JsonVariant value, uint8_t depth) {
    if (!depth || done() || value.isUnbound()) return false;
    const uint8_t initial = *cursor_++;
    const uint8_t major = initial >> 5;
    const uint8_t info = initial & 31;
    uint64_t number;
    if (!argument(info, number)) return false;
    switch (major) {
      case 0: return value.set(number);
      case 1:
        return number <= INT64_MAX && value.set(-1 - static_cast<int64_t>(number));
      case 3: {
        JsonString text;
        return string(number, text) && value.set(text);
      }
      case 4: {
        if (number > remaining()) return false;
        JsonArray array = value.to<JsonArray>();
        for (uint64_t i = 0; i < number; ++i) {
          if (!read(array.add<JsonVariant>(), depth - 1)) return false;
        }
        return !array.isNull();
      }
      case 5: {
        if (number > remaining() / 2) return false;
        JsonObject object = value.to<JsonObject>();
        for (uint64_t i = 0; i < number; ++i) {
          if (done()) return false;
          const uint8_t keyHead = *cursor_++;
          uint64_t length;
          JsonString key;
          if ((keyHead >> 5) != 3 || !argument(keyHead & 31, length) || !string(length, key)) return false;
          const size_t count = object.size();
          if (!read(object[key].to<JsonVariant>(), depth - 1) || object.size() != count + 1) return false;
        }
        return !object.isNull();
      }
      case 7: return simple(value, info, number);
      default: return false;
    }
  }

 private:
  size_t remaining() const { return end_ - cursor_; }

  bool argument(uint8_t info, uint64_t& value) {
    value = info;
    if (info < 24) return true;
    if (info > 27) return false;
    const size_t count = size_t(1) << (info - 24);
    if (count > remaining()) return false;
    value = 0;
    for (size_t i = 0; i < count; ++i) value = (value << 8) | *cursor_++;
    return true;
  }

  bool string(uint64_t length, JsonString& value) {
    if (length > remaining()) return false;
    value = JsonString(reinterpret_cast<const char*>(cursor_), static_cast<size_t>(length));
    cursor_ += length;
    return true;
  }

  bool simple(JsonVariant value, uint8_t info, uint64_t bits) {
    if (info == 20 || info == 21) return value.set(info == 21);
    if (info == 22) return value.set(nullptr);
    double number;
    if (info == 25) {
      const int exponent = (bits >> 10) & 31;
      if (exponent == 31) return false;
      const unsigned fraction = bits & 1023;
      number = std::ldexp(double(fraction + (exponent ? 1024 : 0)), exponent ? exponent - 25 : -24);
      if (bits & 0x8000) number = -number;
    } else if (info == 26) {
      const uint32_t word = bits;
      float single;
      memcpy(&single, &word, sizeof(single));
      number = single;
    } else if (info == 27) {
      memcpy(&number, &bits, sizeof(number));
    } else {
      return false;
    }
    return std::isfinite(number) && value.set(number);
  }

  const uint8_t* cursor_;
  const uint8_t* end_;
};
}  // namespace

bool readCbor(const wire::Packet& packet, JsonDocument& doc) {
  if (packet.size > sizeof(packet.payload)) return false;
  CborReader reader(packet.payload, packet.size);
  return reader.read(doc.to<JsonVariant>(), kMaxNesting) &&
         reader.done() && doc.is<JsonObject>() && !doc.overflowed();
}

bool writeCbor(JsonVariantConst value, wire::Packet& packet) {
  CborWriter writer(packet.payload);
  if (!value.is<JsonObjectConst>() || !writer.write(value, kMaxNesting)) return false;
  packet.size = writer.size();
  return true;
}

bool readJson(const wire::Packet& packet, JsonDocument& doc) {
  if (!packet.size || packet.size > sizeof(packet.payload)) return false;
  return !deserializeJson(doc, reinterpret_cast<const char*>(packet.payload), packet.size,
                          DeserializationOption::NestingLimit(kMaxNesting)) &&
         doc.is<JsonObject>() && !doc.overflowed();
}

bool writeJson(JsonVariantConst value, wire::Packet& packet) {
  if (!value.is<JsonObjectConst>()) return false;
  const size_t length = measureJson(value);
  if (length > sizeof(packet.payload)) return false;
  if (serializeJson(value, reinterpret_cast<char*>(packet.payload), sizeof(packet.payload)) != length)
    return false;
  packet.size = length;
  return true;
}
}  // namespace payload
