#include "EXNetworkUdp.h"
#include "DIAG.h"

#if __has_include(<freertos/semphr.h>)
    #include <freertos/semphr.h>
    SemaphoreHandle_t udpMutex = xSemaphoreCreateMutex();
    class UdpQueueLock {
    public:
        UdpQueueLock() { xSemaphoreTake(udpMutex, portMAX_DELAY); }
        ~UdpQueueLock() { xSemaphoreGive(udpMutex); }
    };
#else
 #warning "FreeRTOS semaphores not available, UDP queue will not be thread-safe"
  // STM32 or other platforms without FreeRTOS mutexes
  class UdpQueueLock {
  public:
    UdpQueueLock() {}
    ~UdpQueueLock() {}
  };
#endif

EXNetworkUdpPacket * EXNetworkUdp::firstPendingPacket = nullptr;
EXNetworkUdpPacket * EXNetworkUdp::firstRecycledPacket = nullptr;
EXNetworkUdpPacket * EXNetworkUdp::currentlyProcessingPacket = nullptr;
int16_t EXNetworkUdp::udpQueueLength = 0;

void EXNetworkUdp::addPacket(bool incoming,const IPAddress ip, uint16_t port, const unsigned char * data, int16_t length) {
  if (!data || length > EXNETWORK_UDP_MAX_SIZE) {
    DIAG(F("Invalid UDP packet length %d"), (int)length);
    return;
  }
  UdpQueueLock lock; // unlocks when goes out of scope

  // search pending packets for a matching port, ipaddress and incoming flag
  EXNetworkUdpPacket * lastPacket = nullptr;
  for (auto p = firstPendingPacket; p != nullptr; p = p->next) {
    lastPacket = p;
    if (port == p->port && 
        ip == p->remoteIP && 
        incoming == p->incoming) 
        if ((length+p->length) < EXNETWORK_UDP_MAX_SIZE) {
          // yes we can append this packet to an existing one       
          memcpy(&p->data[p->length], data, length);
          p->length += length;
          p->data[p->length] = 0; // null-terminate for safety
          return; // packet appended, no chain change
        }
    }

// Allocate a new packet, either from the recycled list or by creating a new one
    auto newpacket = firstRecycledPacket;
    if (newpacket) firstRecycledPacket = newpacket->next;
    else {
      newpacket = new EXNetworkUdpPacket;
      udpQueueLength++;
      DIAG("UDP Queue length=%d", udpQueueLength);
    }
    newpacket->next = nullptr;
    newpacket->length = length;
    newpacket->incoming = incoming;
    newpacket->remoteIP = ip;
    newpacket->port = port;
    memcpy(newpacket->data, data, length);
    newpacket->data[newpacket->length] = 0; // null-terminate for safety
    // chain to end of queue
    if (lastPacket) lastPacket->next = newpacket;
    else firstPendingPacket = newpacket;
}


EXNetworkUdpPacket* EXNetworkUdp::getPendingPacket() {
  UdpQueueLock lock;
  // recycle previous current packet if any
  if (currentlyProcessingPacket) {
    currentlyProcessingPacket->next = firstRecycledPacket;
    firstRecycledPacket = currentlyProcessingPacket;
    currentlyProcessingPacket = nullptr;
  }

  // get the first pending packet
  auto p = firstPendingPacket;
  if (p) {
    currentlyProcessingPacket = p;
    firstPendingPacket = p->next;
  }
    return p;
}
