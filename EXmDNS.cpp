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

#include <Arduino.h>
#include "EthernetInterface.h"
#ifdef DO_MDNS
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
}
MDNS::~MDNS() {
  _udp->stop();
  if (_name) free(_name);
  if (_serviceName) free(_serviceName);
  if (_serviceProto) free(_serviceProto);
}
int MDNS::begin(const IPAddress& ip, char* name) {
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
  // we ignore proto, assume TCP
  (void)proto;
  _serviceName = (char *)malloc(strlen(name) + 2);
  byte n;
  for(n = 0; n<strlen(name); n++)
    _serviceName[n+1] = name[n];
  _serviceName[n+1] = '\0';
  _serviceName[0] = '.';
  _serviceProto = NULL; //to be filled in
  dotToLen(_serviceName, &_serviceProto);
  _servicePort = port;
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
  dnsHeader.answerCount = HTONS(4 /*5 if TXT but we do not do that */);
  _udp->write((uint8_t*)&dnsHeader, sizeof(DNSHeader_t));

  // rr #1, the PTR record from generic _services.x.local to service.x.local
  _udp->write((uint8_t*)dns_rr_services, sizeof(dns_rr_services));

  byte buf[10];
  buf[0] = 0x00;
  buf[1] = 0x0c;                           //PTR
  buf[2] = 0x00;
  buf[3] = 0x01;                           //IN
  *((uint32_t*)(buf+4)) = HTONL(120); //TTL in sec
  *((uint16_t*)(buf+8)) = HTONS( _serviceProto[0] + 1 + strlen(dns_rr_tcplocal) + 1);
  _udp->write(buf, 10);

  _udp->write(_serviceProto,_serviceProto[0]+1);
  _udp->write(dns_rr_tcplocal, strlen(dns_rr_tcplocal)+1);

  // rr #2, the PTR record from proto.x to name.proto.x
  _udp->write(_serviceProto,_serviceProto[0]+1);
  _udp->write(dns_rr_tcplocal, strlen(dns_rr_tcplocal)+1);
  *((uint16_t*)(buf+8)) = HTONS(strlen(_serviceName) + strlen(dns_rr_tcplocal) + 1); // recycle most of buf
  _udp->write(buf, 10);

  _udp->write(_serviceName, strlen(_serviceName));
  _udp->write(dns_rr_tcplocal, strlen(dns_rr_tcplocal)+1);
  // rr #3, the SRV record for the service that points to local name
  _udp->write(_serviceName, strlen(_serviceName));
  _udp->write(dns_rr_tcplocal, strlen(dns_rr_tcplocal)+1);

  buf[1] = 0x21;                                  // recycle most of buf but here SRV
  buf[2] = 0x80;                                  // cache flush
  *((uint16_t*)(buf+8)) = HTONS(strlen(_name) + strlen(dns_rr_local) + 1 + 6);
  _udp->write(buf, 10);

  byte srv[6];
  // priority and weight
  srv[0] = srv[1] = srv[2] = srv[3] = 0;
  // port
  *((uint16_t*)(srv+4)) = HTONS(_servicePort);
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
  
  _udp->endPacket();
  _udp->flush();
  // 
}
#endif //DO_MDNS
