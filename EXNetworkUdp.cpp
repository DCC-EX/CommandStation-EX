#include "EXNetwork.h"

#ifdef HAS_EXNETWORK
#include "EXNetworkUdp.h"
#include "DIAG.h"

#ifdef ARDUINO_ARCH_ESP32
#include <freertos/semphr.h>
static SemaphoreHandle_t udpMutex = nullptr;
class UdpQueueLock {
public:
  UdpQueueLock() { xSemaphoreTake(udpMutex, portMAX_DELAY); }
  ~UdpQueueLock() { xSemaphoreGive(udpMutex); }
};
#else
class UdpQueueLock {
public:
  UdpQueueLock() {}
};
#endif

namespace {
constexpr size_t UDP_QUEUE_CAPACITY = 8;
constexpr size_t UDP_BATCH_MAX_SIZE = 1460;
EXNetworkUdpPacket packets[UDP_QUEUE_CAPACITY];
EXNetworkUdpPacket processingPacket;
size_t head = 0;
size_t count = 0;
unsigned long dropped = 0;
}

bool EXNetworkUdp::setup() {
#ifdef ARDUINO_ARCH_ESP32
  if (!udpMutex) udpMutex = xSemaphoreCreateMutex();
  if (!udpMutex) {
    DIAG(F("UDP queue mutex allocation failed"));
    return false;
  }
#endif
  return true;
}

bool EXNetworkUdp::addPacket(bool incoming, const IPAddress &ip, uint16_t port,
                             const unsigned char *data, size_t length) {
  if (!data || !length || length > EXNETWORK_UDP_MAX_SIZE) return false;
  UdpQueueLock lock;
  if (!incoming && count) {
    auto &tail = packets[(head + count - 1) % UDP_QUEUE_CAPACITY];
    if (!tail.incoming && tail.remoteIP == ip && tail.port == port &&
        tail.length + length <= UDP_BATCH_MAX_SIZE) {
      memcpy(tail.data + tail.length, data, length);
      tail.length += length;
      tail.data[tail.length] = 0;
      return true;
    }
  }
  if (count == UDP_QUEUE_CAPACITY) {
    ++dropped;
    return false;
  }
  auto &packet = packets[(head + count) % UDP_QUEUE_CAPACITY];
  packet.length = length;
  packet.incoming = incoming;
  packet.remoteIP = ip;
  packet.port = port;
  memcpy(packet.data, data, length);
  packet.data[length] = 0;
  ++count;
  return true;
}

void EXNetworkUdp::clear() {
  UdpQueueLock lock;
  head = 0;
  count = 0;
}

unsigned long EXNetworkUdp::takeDroppedCount() {
  UdpQueueLock lock;
  unsigned long result = dropped;
  dropped = 0;
  return result;
}

EXNetworkUdpPacket *EXNetworkUdp::getPendingPacket() {
  UdpQueueLock lock;
  if (!count) return nullptr;
  processingPacket = packets[head];
  head = (head + 1) % UDP_QUEUE_CAPACITY;
  --count;
  return &processingPacket;
}
#endif
