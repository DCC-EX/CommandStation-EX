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

#ifdef ARDUINO_ARCH_STM32
#include <Arduino.h>
#include "EthernetInterface.h"

#include "EXmDNS.h"

// fixed values for mDNS
static IPAddress mdnsMulticastIPAddr = IPAddress(224, 0, 0, 251);
#define MDNS_SERVER_PORT 5353

// dotToLen()
// converts stings of form ".foo.barbar.x" to a string with the
// dots replaced with lenght. So string above  would result in
// "\x03foo\x06barbar\x01x" in C notation. If not NULL, *substr
// will point to the beginning of the last component, in this
// example that would be "\x01x".
//
static void dotToLen(char *str, char **substr) {
  char *dotplace = NULL;
  char *s;
  byte charcount = 0;
  for (s = str;/*see break*/ ; s++) {
    if (*s == '.' || *s == '\0') {
      // take care of accumulated
      if (dotplace != NULL && charcount != 0) {
	*dotplace = charcount;
      }
      if (*s == '\0')
	break;
      if (substr && *s == '.')
	*substr = s;
      // set new values
      dotplace = s;
      charcount = 0;
    } else {
      charcount++;
    }
  }
}

MDNS::MDNS(EthernetUDP& udp) {
  _udp = &udp;
  _name = NULL;
  _serviceCount = 0;
  for (uint8_t i = 0; i < MAX_SERVICE_RECORDS; i++) {
    _services[i] = {NULL, MDNSServiceTCP, 0, NULL, 0};
  }
}
MDNS::~MDNS() {
  _udp->stop();
  if (_name) free(_name);
  for (uint8_t i = 0; i < _serviceCount; i++) {
    free(_services[i].name);
    free(_services[i].text);
  }
}
int MDNS::begin(const IPAddress& ip, const char* name) {
  // if we were called very soon after the board was booted, we need to give the
  // EthernetShield (WIZnet) some time to come up. Hence, we delay until millis() is at
  // least 3000. This is necessary, so that if we need to add a service record directly
  // after begin, the announce packet does not get lost in the bowels of the WIZnet chip.
  //while (millis() < 3000) 
  //  delay(100);
  
  _ipAddress = ip;
  _name = (char *)malloc(strlen(name)+2);
  byte n;
  for(n = 0; n<strlen(name); n++)
    _name[n+1] = name[n];
  _name[n+1] = '\0';
  _name[0] = '.';
  dotToLen(_name, NULL);
  return _udp->beginMulticast(mdnsMulticastIPAddr, MDNS_SERVER_PORT);
}

int MDNS::addServiceRecord(const char* name, uint16_t port, MDNSServiceProtocol_t proto) {
  if (!name || !name[0]) return 0;

  for (uint8_t i = 0; i < _serviceCount; i++) {
    if (_services[i].proto == proto && strcmp(_services[i].name, name) == 0) {
      _services[i].port = port;
      return 1;
    }
  }

  if (_serviceCount >= MAX_SERVICE_RECORDS) return 0;
  ServiceRecord &service = _services[_serviceCount++];
  service.name = strdup(name);
  if (!service.name) {
    _serviceCount--;
    return 0;
  }
  service.proto = proto;
  service.port = port;
  service.text = NULL;
  service.textLength = 0;
  return 1;
}

int MDNS::addTextRecord(const char* name, MDNSServiceProtocol_t proto,
                        const char* key, const char* value) {
  if (!name || !key || !value) return 0;

  ServiceRecord *service = NULL;
  for (uint8_t i = 0; i < _serviceCount; i++) {
    if (_services[i].proto == proto && strcmp(_services[i].name, name) == 0) {
      service = &_services[i];
      break;
    }
  }
  if (!service) return 0;

  size_t keyLength = strlen(key);
  size_t valueLength = strlen(value);
  size_t textLength = keyLength + 1 + valueLength;
  if (textLength > 255 || service->textLength + textLength + 1 > 512) return 0;

  uint8_t *text = (uint8_t *)realloc(service->text,
                                     service->textLength + textLength + 1);
  if (!text) return 0;

  service->text = text;
  uint8_t *record = service->text + service->textLength;
  record[0] = textLength;
  memcpy(record + 1, key, keyLength);
  record[1 + keyLength] = '=';
  memcpy(record + 2 + keyLength, value, valueLength);
  service->textLength += textLength + 1;
  return 1;
}

static char dns_rr_services[]   = "\x09_services\x07_dns-sd\x04_udp\x05local";
static char dns_rr_tcplocal[]   = "\x04_tcp\x05local"; 
static char *dns_rr_local       = dns_rr_tcplocal + dns_rr_tcplocal[0] + 1;

typedef struct _DNSHeader_t 
{
  uint16_t    xid;
  uint16_t  flags; // flags condensed
  uint16_t    queryCount;
  uint16_t    answerCount;
  uint16_t    authorityCount;
  uint16_t    additionalCount;
} __attribute__((__packed__)) DNSHeader_t;

//
// MDNS::run()
// This broadcasts whatever we got evey BROADCASTTIME seconds.
// Why? Too much brokenness i all mDNS implementations available
//
void MDNS::run() {
  if (_serviceCount == 0) return;

  ServiceRecord &service = _services[_serviceCount - 1];
  char serviceName[64];
  size_t serviceNameLength = strlen(service.name);
  if (serviceNameLength > sizeof(serviceName) - 2) return;
  serviceName[0] = serviceNameLength;
  memcpy(serviceName + 1, service.name, serviceNameLength);
  serviceName[serviceNameLength + 1] = '\0';

  static const char tcpProto[] = "\x04_tcp\x05local";
  static const char udpProto[] = "\x04_udp\x05local";
  const char *serviceProto = service.proto == MDNSServiceUDP ? udpProto : tcpProto;
  const char *serviceLocal = serviceProto + serviceProto[0] + 1;

  static long int lastrun = BROADCASTTIME * 1000UL;
  unsigned long int now = millis();
  if (!(now - lastrun > BROADCASTTIME * 1000UL)) {
    return;
  }
  lastrun = now;
  DNSHeader_t dnsHeader = {0, 0, 0, 0, 0, 0};
  // DNSHeader_t dnsHeader = { 0 };

  _udp->beginPacket(mdnsMulticastIPAddr, MDNS_SERVER_PORT);

  // dns header
  dnsHeader.flags = HTONS((uint16_t)0x8400); // Response, authorative
  dnsHeader.answerCount = HTONS(service.textLength ? 5 : 4);
  _udp->write((uint8_t*)&dnsHeader, sizeof(DNSHeader_t));

  // rr #1, the PTR record from generic _services.x.local to service.x.local
  _udp->write((uint8_t*)dns_rr_services, sizeof(dns_rr_services));

  byte buf[10];
  buf[0] = 0x00;
  buf[1] = 0x0c;                           //PTR
  buf[2] = 0x00;
  buf[3] = 0x01;                           //IN
  *((uint32_t*)(buf+4)) = HTONL(120); //TTL in sec
  *((uint16_t*)(buf+8)) = HTONS(serviceProto[0] + 1 + strlen(serviceLocal) + 1);
  _udp->write(buf, 10);

  _udp->write(serviceProto, serviceProto[0] + 1);
  _udp->write(serviceLocal, strlen(serviceLocal) + 1);

  // rr #2, the PTR record from proto.x to name.proto.x
  _udp->write(serviceProto, serviceProto[0] + 1);
  _udp->write(serviceLocal, strlen(serviceLocal) + 1);
  *((uint16_t*)(buf+8)) = HTONS(strlen(serviceName) + strlen(serviceLocal) + 1); // recycle most of buf
  _udp->write(buf, 10);

  _udp->write(serviceName, serviceName[0] + 1);
  _udp->write(serviceLocal, strlen(serviceLocal) + 1);
  // rr #3, the SRV record for the service that points to local name
  _udp->write(serviceName, serviceName[0] + 1);
  _udp->write(serviceLocal, strlen(serviceLocal) + 1);

  buf[1] = 0x21;                                  // recycle most of buf but here SRV
  buf[2] = 0x80;                                  // cache flush
  *((uint16_t*)(buf+8)) = HTONS(strlen(_name) + strlen(dns_rr_local) + 1 + 6);
  _udp->write(buf, 10);

  byte srv[6];
  // priority and weight
  srv[0] = srv[1] = srv[2] = srv[3] = 0;
  // port
  *((uint16_t*)(srv+4)) = HTONS(service.port);
  _udp->write(srv, 6);
  // target
  _udp->write(_name, _name[0]+1);
  _udp->write(dns_rr_local, strlen(dns_rr_local)+1);

  // rr #4, the A record for the name.local
  _udp->write(_name, _name[0]+1);
  _udp->write(dns_rr_local, strlen(dns_rr_local)+1);
  
  buf[1] = 0x01;                                  // recycle most of buf but here A
  *((uint16_t*)(buf+8)) = HTONS(4);
  _udp->write(buf, 10);
  byte ip[4];
  ip[0] = _ipAddress[0];
  ip[1] = _ipAddress[1];
  ip[2] = _ipAddress[2];
  ip[3] = _ipAddress[3];
  _udp->write(ip, 4);

  // rr #5, the TXT record containing service metadata.
  if (service.textLength) {
    _udp->write(serviceName, serviceName[0] + 1);
    buf[1] = 0x10; // TXT
    buf[2] = 0x80; // cache flush
    *((uint32_t *)(buf + 4)) = HTONL(120);
    *((uint16_t *)(buf + 8)) = HTONS(service.textLength);
    _udp->write(buf, 10);
    _udp->write(service.text, service.textLength);
  }
  
  _udp->endPacket();
  _udp->flush();
  // 
}
#endif // ARDUINO_ARCH_STM32
