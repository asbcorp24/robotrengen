#include <BridgeProtocol.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <vector>

namespace {
uint32_t randomState = 0x12345678;
uint32_t random32() {
  randomState ^= randomState << 13;
  randomState ^= randomState >> 17;
  randomState ^= randomState << 5;
  return randomState;
}
wire::Packet sample(size_t size) {
  wire::Packet packet;
  packet.type = wire::Type::Telemetry;
  packet.sequence = 0x12345678;
  packet.size = size;
  for (size_t i = 0; i < size; ++i) packet.payload[i] = random32() & 0xff;
  return packet;
}
void same(const wire::Packet& a, const wire::Packet& b) {
  assert(a.type == b.type && a.sequence == b.sequence && a.size == b.size);
  assert(memcmp(a.payload, b.payload, a.size) == 0);
}
std::vector<uint8_t> raw(const wire::Packet& p) {
  std::vector<uint8_t> bytes(wire::kMaxRaw);
  const size_t size = wire::encodeRaw(p, bytes.data(), bytes.size());
  assert(size);
  bytes.resize(size);
  return bytes;
}
std::vector<uint8_t> frame(const wire::Packet& p) {
  const auto bytes = raw(p);
  std::vector<uint8_t> encoded(wire::kMaxEncoded + 1);
  const size_t size = wire::cobsEncode(bytes.data(), bytes.size(), encoded.data(), encoded.size() - 1);
  assert(size);
  encoded.resize(size + 1);
  encoded[size] = 0;
  return encoded;
}
std::vector<uint8_t> fragment(const std::vector<uint8_t>& bytes, uint16_t id,
                              size_t offset, size_t count) {
  std::vector<uint8_t> out(wire::kFragmentHeader + count);
  wire::write16(out.data(), id);
  wire::write16(out.data() + 2, bytes.size());
  wire::write16(out.data() + 4, offset);
  memcpy(out.data() + wire::kFragmentHeader, bytes.data() + offset, count);
  return out;
}
void testCrcAndWire() {
  const uint8_t text[] = "123456789";
  assert(wire::crc32(text, 9) == 0xcbf43926);
  // Independent Python struct/zlib fixture, also printed by stm32_simulator.py.
  const uint8_t golden[] = {0x01, 0x04, 0, 0, 0x01, 0, 0x01, 0, 0, 0,
                            0xa0, 0xba, 0x4f, 0x23, 0xee};
  wire::Packet request;
  request.type = wire::Type::SnapshotRequest;
  request.sequence = 1;
  request.size = 1;
  request.payload[0] = 0xa0;
  const auto requestRaw = raw(request);
  assert(requestRaw.size() == sizeof(golden));
  assert(memcmp(requestRaw.data(), golden, sizeof(golden)) == 0);
  for (size_t size = 0; size <= wire::kMaxPayload; ++size) {
    const auto p = sample(size);
    auto bytes = raw(p);
    wire::Packet decoded;
    assert(wire::decodeRaw(bytes.data(), bytes.size(), decoded));
    same(p, decoded);
    assert(!wire::decodeRaw(bytes.data(), bytes.size() - 1, decoded));
    bytes[bytes.size() / 2] ^= 1;
    assert(!wire::decodeRaw(bytes.data(), bytes.size(), decoded));
  }
  auto p = sample(12);
  auto bytes = raw(p);
  for (const size_t index : {size_t(0), size_t(2), size_t(4), size_t(5)}) {
    auto bad = bytes;
    bad[index] = 0xff;
    wire::write32(bad.data() + bad.size() - 4, wire::crc32(bad.data(), bad.size() - 4));
    wire::Packet decoded;
    assert(!wire::decodeRaw(bad.data(), bad.size(), decoded));
  }
  uint8_t small[8];
  assert(wire::encodeRaw(p, small, sizeof(small)) == 0);
  p.size = wire::kMaxPayload + 1;
  uint8_t large[wire::kMaxRaw];
  assert(wire::encodeRaw(p, large, sizeof(large)) == 0);
  puts("PASS CRC vector, packet lengths, flags, version and corruption");
}
void testCobs() {
  uint8_t input[wire::kMaxRaw], encoded[wire::kMaxEncoded], decoded[wire::kMaxRaw];
  for (size_t size = 0; size <= sizeof(input); ++size) {
    for (unsigned pattern = 0; pattern < 3; ++pattern) {
      for (size_t i = 0; i < size; ++i)
        input[i] = pattern == 0 ? 0 : (pattern == 1 ? 0xff : random32() & 0xff);
      const size_t n = wire::cobsEncode(input, size, encoded, sizeof(encoded));
      assert(n > 0 && n <= wire::kMaxEncoded);
      for (size_t i = 0; i < n; ++i) assert(encoded[i]);
      assert(wire::cobsDecode(encoded, n, decoded, sizeof(decoded)) == size);
      assert(memcmp(input, decoded, size) == 0);
      if (size) assert(wire::cobsDecode(encoded, n, decoded, size - 1) == 0);
      assert(wire::cobsEncode(input, size, encoded, n - 1) == 0);
    }
  }
  const uint8_t bad1[] = {3, 1}, bad2[] = {2, 0}, bad3[] = {0};
  assert(!wire::cobsDecode(bad1, sizeof(bad1), decoded, sizeof(decoded)));
  assert(!wire::cobsDecode(bad2, sizeof(bad2), decoded, sizeof(decoded)));
  assert(!wire::cobsDecode(bad3, sizeof(bad3), decoded, sizeof(decoded)));
  puts("PASS COBS all sizes, zero/nonzero/random patterns, capacity and malformed input");
}
void testUart() {
  wire::UartDecoder decoder;
  const auto p = sample(wire::kMaxPayload);
  const auto bytes = frame(p);
  wire::Packet decoded;
  for (const uint8_t b : bytes) {
    const auto result = decoder.feed(b, 100, decoded);
    assert(result == (b ? wire::FeedResult::None : wire::FeedResult::Complete));
  }
  same(p, decoded);
  assert(decoder.feed(0, 100, decoded) == wire::FeedResult::None);
  for (size_t i = 0; i < wire::kMaxEncoded + 10; ++i) decoder.feed(1, 101, decoded);
  assert(decoder.feed(0, 101, decoded) == wire::FeedResult::Rejected);
  for (const uint8_t b : bytes) decoder.feed(b, 102, decoded);
  same(p, decoded);
  decoder.feed(4, 0xfffffff0u, decoded);
  decoder.expire(0x00000100u, 250);
  assert(decoder.feed(0, 0x00000101u, decoded) == wire::FeedResult::Rejected);
  for (const uint8_t b : bytes) decoder.feed(b, 0x00000102u, decoded);
  same(p, decoded);
  auto bad = bytes;
  bad[bad.size() / 2] ^= 0x80;
  for (size_t i = 0; i < bad.size(); ++i) {
    const auto result = decoder.feed(bad[i], 500, decoded);
    if (i + 1 == bad.size()) assert(result == wire::FeedResult::Rejected);
  }
  for (const uint8_t b : bytes) decoder.feed(b, 501, decoded);
  same(p, decoded);
  puts("PASS UART overflow, timeout across millis wrap, corruption and resynchronization");
}
void testFragments() {
  wire::Reassembler assembler;
  wire::Packet decoded;
  for (const size_t payload : {size_t(0), size_t(1), size_t(100), wire::kMaxPayload}) {
    const auto p = sample(payload);
    const auto bytes = raw(p);
    for (const size_t chunk : {size_t(14), size_t(17), size_t(238)}) {
      assembler.reset();
      size_t offset = 0;
      while (offset < bytes.size()) {
        const size_t count = bytes.size() - offset < chunk ? bytes.size() - offset : chunk;
        const auto f = fragment(bytes, 12, offset, count);
        const auto result = assembler.feed(f.data(), f.size(), 100, decoded);
        offset += count;
        assert(result == (offset == bytes.size() ? wire::FeedResult::Complete : wire::FeedResult::None));
      }
      same(p, decoded);
    }
  }
  const auto bytes = raw(sample(80));
  const auto first = fragment(bytes, 1, 0, 14);
  const auto second = fragment(bytes, 1, 14, 14);
  auto wrongId = second;
  wire::write16(wrongId.data(), 2);
  assert(assembler.feed(first.data(), first.size(), 10, decoded) == wire::FeedResult::None);
  assert(assembler.feed(wrongId.data(), wrongId.size(), 11, decoded) == wire::FeedResult::Rejected);
  assert(assembler.feed(second.data(), second.size(), 12, decoded) == wire::FeedResult::Rejected);
  assembler.feed(first.data(), first.size(), 20, decoded);
  assembler.feed(second.data(), second.size(), 21, decoded);
  assert(assembler.feed(second.data(), second.size(), 22, decoded) == wire::FeedResult::Rejected);
  assembler.feed(first.data(), first.size(), 0xfffffff0u, decoded);
  assembler.expire(0x00000400u, 1000);
  assert(assembler.feed(second.data(), second.size(), 0x00000401u, decoded) == wire::FeedResult::Rejected);
  auto invalidTotal = first;
  wire::write16(invalidTotal.data() + 2, wire::kMaxRaw + 1);
  assert(assembler.feed(invalidTotal.data(), invalidTotal.size(), 2000, decoded) == wire::FeedResult::Rejected);
  const uint8_t tiny[6]{};
  assert(assembler.feed(tiny, sizeof(tiny), 2000, decoded) == wire::FeedResult::Rejected);
  puts("PASS BLE fragmentation at MTU 23/247, order, duplicates, message ID, timeout and bounds");
}
void testQueueAndRouting() {
  wire::Queue<int, 2> q;
  int v;
  assert(!q.pop(v));
  assert(q.push(10) && q.push(20) && !q.push(30));
  assert(q.pop(v) && v == 10);
  assert(q.push(30));
  assert(q.pop(v) && v == 20);
  assert(q.pop(v) && v == 30);
  q.push(1);
  q.clear();
  assert(q.size() == 0 && !q.pop(v));
  assert(wire::allowedFromPhone(wire::Type::Stop));
  assert(!wire::allowedFromPhone(wire::Type::Telemetry));
  assert(!wire::allowedFromPhone(wire::Type::LinkState));
  assert(wire::allowedFromController(wire::Type::CommandResult));
  assert(!wire::allowedFromController(wire::Type::Command));
  puts("PASS queue wrap/full/clear and directional type filters");
}
}  // namespace

int main() {
  testCrcAndWire();
  testCobs();
  testUart();
  testFragments();
  testQueueAndRouting();
  puts("All protocol tests passed.");
}
