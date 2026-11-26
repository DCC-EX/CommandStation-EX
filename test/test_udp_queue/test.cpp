#ifdef UDP_QUEUE_HOST_TEST
#include <cstring>
#include <thread>
#include <unity.h>

#define EXNetwork_h
#define HAS_EXNETWORK
#define DIAG_h
#define DIAG(...) ((void)0)
#define ARDUINO_ARCH_ESP32
#include "../../EXNetworkUdp.cpp"

static const IPAddress first(1), second(2);
static const byte a[] = "<a>", b[] = "<b>", c[] = "<c>";

void setUp() {
  EXNetworkUdp::clear();
  EXNetworkUdp::takeDroppedCount();
}

void tearDown() {}

static void expectPacket(const char *data) {
  auto *packet = EXNetworkUdp::getPendingPacket();
  TEST_ASSERT_NOT_NULL(packet);
  TEST_ASSERT_EQUAL_STRING(data, reinterpret_cast<char *>(packet->data));
}

void test_incoming_packets_keep_order_and_boundaries() {
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(true, first, 2560, a, sizeof(a) - 1));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(true, second, 2560, b, sizeof(b) - 1));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(true, first, 2560, c, sizeof(c) - 1));
  expectPacket("<a>");
  expectPacket("<b>");
  expectPacket("<c>");
  TEST_ASSERT_NULL(EXNetworkUdp::getPendingPacket());
}

void test_outgoing_batching_preserves_order() {
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, a, sizeof(a) - 1));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, b, sizeof(b) - 1));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, second, 2560, c, sizeof(c) - 1));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, c, sizeof(c) - 1));
  expectPacket("<a><b>");
  expectPacket("<c>");
  expectPacket("<c>");
  TEST_ASSERT_NULL(EXNetworkUdp::getPendingPacket());
}

void test_packet_size_and_batch_limit() {
  byte payload[EXNETWORK_UDP_MAX_SIZE + 1] = {};
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, payload, 730));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, payload, 730));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(false, first, 2560, payload, 1));
  auto *packet = EXNetworkUdp::getPendingPacket();
  TEST_ASSERT_NOT_NULL(packet);
  TEST_ASSERT_EQUAL_UINT(1460, packet->length);
  packet = EXNetworkUdp::getPendingPacket();
  TEST_ASSERT_NOT_NULL(packet);
  TEST_ASSERT_EQUAL_UINT(1, packet->length);
  TEST_ASSERT_NULL(EXNetworkUdp::getPendingPacket());
  TEST_ASSERT_FALSE(EXNetworkUdp::addPacket(true, first, 2560, payload, sizeof(payload)));
  TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(true, first, 2560, payload, EXNETWORK_UDP_MAX_SIZE));
  EXNetworkUdp::clear();
  TEST_ASSERT_NULL(EXNetworkUdp::getPendingPacket());
}

void test_full_queue_and_clear() {
  for (int i = 0; i < 8; ++i)
    TEST_ASSERT_TRUE(EXNetworkUdp::addPacket(true, first, 2560, a, sizeof(a) - 1));
  TEST_ASSERT_FALSE(EXNetworkUdp::addPacket(false, first, 2560, b, sizeof(b) - 1));
  TEST_ASSERT_EQUAL_UINT(1, EXNetworkUdp::takeDroppedCount());
  auto *current = EXNetworkUdp::getPendingPacket();
  TEST_ASSERT_NOT_NULL(current);
  EXNetworkUdp::clear();
  TEST_ASSERT_EQUAL_STRING("<a>", reinterpret_cast<char *>(current->data));
  TEST_ASSERT_NULL(EXNetworkUdp::getPendingPacket());
}

void test_concurrent_producers_preserve_order() {
  constexpr int messages = 1000;
  auto producer = [&](const IPAddress &ip) {
    for (int i = 0; i < messages; ++i) {
      byte data[sizeof(i)];
      memcpy(data, &i, sizeof(i));
      while (!EXNetworkUdp::addPacket(true, ip, 2560, data, sizeof(data)))
        std::this_thread::yield();
    }
  };
  std::thread one(producer, first), two(producer, second);
  int nextFirst = 0, nextSecond = 0;
  bool ordered = true;
  while (nextFirst + nextSecond < 2 * messages) {
    auto *packet = EXNetworkUdp::getPendingPacket();
    if (!packet) {
      std::this_thread::yield();
      continue;
    }
    int number;
    memcpy(&number, packet->data, sizeof(number));
    if (packet->remoteIP == first) ordered &= number == nextFirst++;
    else {
      ordered &= packet->remoteIP == second;
      ordered &= number == nextSecond++;
    }
  }
  one.join();
  two.join();
  TEST_ASSERT_TRUE(ordered);
}

int main() {
  if (!EXNetworkUdp::setup()) return 1;
  UNITY_BEGIN();
  RUN_TEST(test_incoming_packets_keep_order_and_boundaries);
  RUN_TEST(test_outgoing_batching_preserves_order);
  RUN_TEST(test_packet_size_and_batch_limit);
  RUN_TEST(test_full_queue_and_clear);
  RUN_TEST(test_concurrent_producers_preserve_order);
  return UNITY_END();
}
#endif
