/*
 *  © 2024 Harald Barth
 *  © 2024 Paul M. Antoine
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "defines.h"
#if defined(ARDUINO_ARCH_STM32) || (defined(ARDUINO_ARCH_ESP32) && ETHERNET_ON)
#define BROADCASTTIME 15 //seconds

// We do this ourselves because every library is different and/or broken...
#define HTONS(x) ((uint16_t)(((x) << 8) | (((x) >> 8) & 0xFF)))
#define HTONL(x) ( ((uint32_t)(x) << 24) | (((uint32_t)(x) << 8) & 0xFF0000) | \
                  (((uint32_t)(x) >> 8) & 0xFF00) | ((uint32_t)(x) >> 24) )

typedef enum _MDNSServiceProtocol_t 
{
  MDNSServiceTCP,
  MDNSServiceUDP
} MDNSServiceProtocol_t;

class MDNS {
public:
  MDNS(EthernetUDP& udp);
  ~MDNS();
  int begin(const IPAddress& ip,  const char* name);
  int addServiceRecord(const char* name, uint16_t port, MDNSServiceProtocol_t proto);
  int addTextRecord(const char* name, MDNSServiceProtocol_t proto,
                    const char* key, const char* value);
  void run();
private:
  EthernetUDP *_udp;
  IPAddress _ipAddress;
  char* _name;
  struct ServiceRecord {
    char* name;
    MDNSServiceProtocol_t proto;
    uint16_t port;
    uint8_t* text;
    uint16_t textLength;
  };
  static const uint8_t MAX_SERVICE_RECORDS = 8;
  ServiceRecord _services[MAX_SERVICE_RECORDS];
  uint8_t _serviceCount;
  void sendServiceAnnounce(ServiceRecord &service);
  void processQueries();
};
#endif // ARDUINO_ARCH_STM32 || (ARDUINO_ARCH_ESP32 && ETHERNET_ON)
