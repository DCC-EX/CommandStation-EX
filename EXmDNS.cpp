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

// decodeName()
// Expands a (possibly compressed) DNS name found at packet[offset] into a
// fully expanded wire-format name (labels + trailing root byte) in out[].
// Returns the offset in the original packet immediately following the name
// (after the pointer, if one was followed), or -1 on error.
static int decodeName(const uint8_t *packet, int packetLen, int offset,
                       uint8_t *out, int outCap, int *outLen) {
  int pos = offset;
  int afterFirst = -1;
  int written = 0;
  int jumps = 0;
  for (;;) {
    if (pos < 0 || pos >= packetLen) return -1;
    uint8_t lenOrPtr = packet[pos];
    if ((lenOrPtr & 0xC0) == 0xC0) {
      if (pos + 1 >= packetLen) return -1;
      if (afterFirst < 0) afterFirst = pos + 2;
      if (++jumps > 10) return -1; // guard against pointer loops
      pos = ((lenOrPtr & 0x3F) << 8) | packet[pos + 1];
      continue;
    }
    if (lenOrPtr & 0xC0) return -1; // reserved label bits
    if (written + 1 > outCap) return -1;
    out[written++] = lenOrPtr;
    if (lenOrPtr == 0) {
      pos += 1;
      break;
    }
    if (pos + 1 + lenOrPtr > packetLen) return -1;
    if (written + lenOrPtr > outCap) return -1;
    memcpy(out + written, packet + pos + 1, lenOrPtr);
    written += lenOrPtr;
    pos += 1 + lenOrPtr;
  }
  *outLen = written;
  return (afterFirst >= 0) ? afterFirst : pos;
}

// namesEqual()
// Case-insensitive comparison of two fully expanded wire-format DNS names.
static bool namesEqual(const uint8_t *a, size_t aLen, const uint8_t *b, size_t bLen) {
  if (aLen != bLen) return false;
  for (size_t i = 0; i < aLen; i++) {
    uint8_t ca = a[i], cb = b[i];
    if (ca >= 'A' && ca <= 'Z') ca += 32;
    if (cb >= 'A' && cb <= 'Z') cb += 32;
    if (ca != cb) return false;
  }
  return true;
}

//
// MDNS::sendServiceAnnounce()
// Sends the PTR/PTR/SRV/A(/TXT) record set for a single registered service.
//
void MDNS::sendServiceAnnounce(ServiceRecord &service) {
  size_t nameLen = strlen(service.name);
  if (nameLen == 0 || nameLen > 62) return; // DNS label limit, minus the "_" prefix

  // "_<name>" as a single length-prefixed DNS label (e.g. "_dcc-ex").
  char typeLabel[64];
  typeLabel[0] = (char)(nameLen + 1);
  typeLabel[1] = '_';
  memcpy(typeLabel + 2, service.name, nameLen);
  size_t typeLabelLen = nameLen + 2;

  static const char tcpProto[] = "\x04_tcp\x05local";
  static const char udpProto[] = "\x04_udp\x05local";
  const char *serviceProto = service.proto == MDNSServiceUDP ? udpProto : tcpProto;
  size_t serviceProtoLen = service.proto == MDNSServiceUDP ? sizeof(udpProto) : sizeof(tcpProto);

  size_t instanceLabelLen = (size_t)(_name[0] + 1);
  size_t serviceTypeLen = typeLabelLen + serviceProtoLen;      // "_name._proto.local\0"
  size_t instanceNameLen = instanceLabelLen + serviceTypeLen;  // "host._name._proto.local\0"

  DNSHeader_t dnsHeader = {0, 0, 0, 0, 0, 0};

  _udp->beginPacket(mdnsMulticastIPAddr, MDNS_SERVER_PORT);

  // dns header
  dnsHeader.flags = HTONS((uint16_t)0x8400); // Response, authorative
  dnsHeader.answerCount = HTONS(service.textLength ? 5 : 4);
  _udp->write((uint8_t*)&dnsHeader, sizeof(DNSHeader_t));

  // rr #1, the PTR record from generic _services.x.local to the service type
  _udp->write((uint8_t*)dns_rr_services, sizeof(dns_rr_services));

  byte buf[10];
  buf[0] = 0x00;
  buf[1] = 0x0c;                           //PTR
  buf[2] = 0x00;
  buf[3] = 0x01;                           //IN
  *((uint32_t*)(buf+4)) = HTONL(120); //TTL in sec
  *((uint16_t*)(buf+8)) = HTONS(serviceTypeLen);
  _udp->write(buf, 10);

  _udp->write(typeLabel, typeLabelLen);
  _udp->write(serviceProto, serviceProtoLen);

  // rr #2, the PTR record from the service type to this instance
  _udp->write(typeLabel, typeLabelLen);
  _udp->write(serviceProto, serviceProtoLen);
  *((uint16_t*)(buf+8)) = HTONS(instanceNameLen); // recycle most of buf
  _udp->write(buf, 10);

  _udp->write(_name, instanceLabelLen);
  _udp->write(typeLabel, typeLabelLen);
  _udp->write(serviceProto, serviceProtoLen);

  // rr #3, the SRV record for the instance that points to local name
  _udp->write(_name, instanceLabelLen);
  _udp->write(typeLabel, typeLabelLen);
  _udp->write(serviceProto, serviceProtoLen);

  buf[1] = 0x21;                                  // recycle most of buf but here SRV
  buf[2] = 0x80;                                  // cache flush
  *((uint16_t*)(buf+8)) = HTONS(6 + instanceLabelLen + strlen(dns_rr_local) + 1);
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
    _udp->write(_name, instanceLabelLen);
    _udp->write(typeLabel, typeLabelLen);
    _udp->write(serviceProto, serviceProtoLen);
    buf[1] = 0x10; // TXT
    buf[2] = 0x80; // cache flush
    *((uint32_t *)(buf + 4)) = HTONL(120);
    *((uint16_t *)(buf + 8)) = HTONS(service.textLength);
    _udp->write(buf, 10);
    _udp->write(service.text, service.textLength);
  }
  
  _udp->endPacket();
  _udp->flush();
}

//
// MDNS::processQueries()
// Answers incoming mDNS queries for our services/hostname immediately,
// instead of waiting for the next periodic announcement.
//
void MDNS::processQueries() {
  if (!_name) return;

  int packetSize = _udp->parsePacket();
  if (packetSize <= 0) return;

  uint8_t packet[512];
  if (packetSize > (int)sizeof(packet)) packetSize = sizeof(packet);
  int len = _udp->read(packet, packetSize);
  if (len < 12) return;

  uint16_t flags = ((uint16_t)packet[2] << 8) | packet[3];
  if (flags & 0x8000) return; // ignore responses, we only answer queries
  uint16_t qdcount = ((uint16_t)packet[4] << 8) | packet[5];
  if (qdcount == 0) return;

  uint8_t respondedMask = 0;
  bool respondedAll = false;
  int pos = 12;

  for (uint16_t q = 0; q < qdcount; q++) {
    uint8_t qname[256];
    int qnameLen = 0;
    int next = decodeName(packet, len, pos, qname, sizeof(qname), &qnameLen);
    if (next < 0 || next + 4 > len) break; // malformed/truncated, stop parsing
    pos = next + 4; // skip QTYPE(2)+QCLASS(2)

    if (respondedAll) continue;

    if (namesEqual(qname, qnameLen, (const uint8_t*)dns_rr_services, sizeof(dns_rr_services))) {
      for (uint8_t s = 0; s < _serviceCount; s++) sendServiceAnnounce(_services[s]);
      respondedAll = true;
      continue;
    }

    size_t instanceLabelLen = (size_t)(_name[0] + 1);
    uint8_t hostName[80];
    memcpy(hostName, _name, instanceLabelLen);
    memcpy(hostName + instanceLabelLen, dns_rr_local, strlen(dns_rr_local) + 1);
    size_t hostNameLen = instanceLabelLen + strlen(dns_rr_local) + 1;
    if (namesEqual(qname, qnameLen, hostName, hostNameLen)) {
      for (uint8_t s = 0; s < _serviceCount; s++) sendServiceAnnounce(_services[s]);
      respondedAll = true;
      continue;
    }

    for (uint8_t s = 0; s < _serviceCount; s++) {
      if (respondedMask & (1 << s)) continue;
      ServiceRecord &service = _services[s];
      size_t nameLen = strlen(service.name);
      if (nameLen == 0 || nameLen > 62) continue;

      char typeLabel[64];
      typeLabel[0] = (char)(nameLen + 1);
      typeLabel[1] = '_';
      memcpy(typeLabel + 2, service.name, nameLen);
      size_t typeLabelLen = nameLen + 2;

      static const char tcpProto[] = "\x04_tcp\x05local";
      static const char udpProto[] = "\x04_udp\x05local";
      const char *serviceProto = service.proto == MDNSServiceUDP ? udpProto : tcpProto;
      size_t serviceProtoLen = service.proto == MDNSServiceUDP ? sizeof(udpProto) : sizeof(tcpProto);

      uint8_t typeName[80];
      memcpy(typeName, typeLabel, typeLabelLen);
      memcpy(typeName + typeLabelLen, serviceProto, serviceProtoLen);
      size_t typeNameLen = typeLabelLen + serviceProtoLen;

      uint8_t instanceName[144];
      memcpy(instanceName, _name, instanceLabelLen);
      memcpy(instanceName + instanceLabelLen, typeName, typeNameLen);
      size_t instanceNameLen = instanceLabelLen + typeNameLen;

      if (namesEqual(qname, qnameLen, typeName, typeNameLen) ||
          namesEqual(qname, qnameLen, instanceName, instanceNameLen)) {
        sendServiceAnnounce(service);
        respondedMask |= (1 << s);
      }
    }
  }
}

//
// MDNS::run()
// Answers queries as soon as they arrive, and also broadcasts unsolicited
// announcements every BROADCASTTIME seconds for clients already browsing.
//
void MDNS::run() {
  processQueries();

  if (_serviceCount == 0) return;

  static long int lastrun = BROADCASTTIME * 1000UL;
  unsigned long int now = millis();
  if (!(now - lastrun > BROADCASTTIME * 1000UL)) {
    return;
  }
  lastrun = now;

  // Announce every registered service, not just the most recently added one.
  for (uint8_t s = 0; s < _serviceCount; s++) {
    sendServiceAnnounce(_services[s]);
  }
}
#endif // ARDUINO_ARCH_STM32 || (ARDUINO_ARCH_ESP32 && ETHERNET_ON)
