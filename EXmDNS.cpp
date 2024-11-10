
#include <Arduino.h>
#include "EthernetInterface.h"
#include "EXmDNS.h"
#include "DIAG.h"
static IPAddress mdnsMulticastIPAddr = IPAddress(224, 0, 0, 251);
#define MDNS_SERVER_PORT 5353

MDNS::MDNS(EthernetUDP& udp) {
  _udp = &udp;
}
MDNS::~MDNS() {
  _udp->stop();
}
int MDNS::begin(const IPAddress& ip, char* name) {
  // if we were called very soon after the board was booted, we need to give the
  // EthernetShield (WIZnet) some time to come up. Hence, we delay until millis() is at
  // least 3000. This is necessary, so that if we need to add a service record directly
  // after begin, the announce packet does not get lost in the bowels of the WIZnet chip.
  //while (millis() < 3000) 
  //  delay(100);
  
  _ipAddress = ip;
  _name = name;
  return _udp->beginMulticast(mdnsMulticastIPAddr, MDNS_SERVER_PORT);
}
int MDNS::addServiceRecord(const char* name, uint16_t port, MDNSServiceProtocol_t proto) {
  // we ignore proto, assume TCP
  _serviceName = (char *)malloc(strlen(name +2));
  DIAG("name %d %s", strlen(name), name);
  byte n;
  for(n = 0; n<strlen(name); n++)
    _serviceName[n+1] = name[n];
  _serviceName[n+1] = '\0';
  //strcpy(&_serviceName[1], name);
  _serviceName[0] = (byte)strlen(name);
  DIAG("sn %d", *_serviceName);
  _servicePort = port;
  DIAG("Sevicename %d %s", strlen(_serviceName), _serviceName);
  return 1;
}

static char dns_rr_services[]   = "\x09_services\x07_dns-sd\x04_udp\x05local";
static char dns_rr_withrottle[] = "\x0b_withrottle\x04_tcp\x05local";
static char *dns_rr_tcplocal    = dns_rr_withrottle + dns_rr_withrottle[0] + 1; // jump over first record
static char *dns_rr_local       = dns_rr_tcplocal + dns_rr_tcplocal[0] + 1;

typedef struct _DNSHeader_t 
{
  uint16_t    xid;
  uint16_t  flags;
  /*
  uint8_t     recursionDesired    : 1;
  uint8_t     truncated           : 1;
  uint8_t     authoritiveAnswer   : 1;
  uint8_t     opCode              : 4;
  uint8_t     queryResponse       : 1;
  uint8_t     responseCode        : 4;
  uint8_t     checkingDisabled    : 1;
  uint8_t     authenticatedData   : 1;
  uint8_t     zReserved           : 1;
  uint8_t     recursionAvailable  : 1;
  */
  uint16_t    queryCount;
  uint16_t    answerCount;
  uint16_t    authorityCount;
  uint16_t    additionalCount;
} __attribute__((__packed__)) DNSHeader_t;

void MDNS::run() {
  static long int lastrun = 10000;
  unsigned long int now = millis();
  if (!(now - lastrun > 10000)) {
    return;
  }
  lastrun = now;
  DNSHeader_t dnsHeader = { 0 };

  _udp->beginPacket(mdnsMulticastIPAddr, MDNS_SERVER_PORT);

  // dns header
  dnsHeader.flags = lwip_htons(0x8400); // Response, authorative
  dnsHeader.answerCount = lwip_htons(4 /*5*/);
  _udp->write((uint8_t*)&dnsHeader, sizeof(DNSHeader_t));
  // rr #1
  _udp->write((uint8_t*)dns_rr_services, sizeof(dns_rr_services));
  byte buf[10];
  buf[0] = 0x00;
  buf[1] = 0x0c;                           //PTR
  buf[2] = 0x00;
  buf[3] = 0x01;                           //IN
  *((uint32_t*)(buf+4)) = lwip_htonl(120); //TTL in sec
  *((uint16_t*)(buf+8)) = lwip_htons(sizeof(dns_rr_withrottle));
  _udp->write(buf, 10);
  _udp->write(dns_rr_withrottle, sizeof(dns_rr_withrottle));
  // rr #2
  _udp->write(dns_rr_withrottle, sizeof(dns_rr_withrottle));
  *((uint16_t*)(buf+8)) = lwip_htons(strlen(_serviceName) + sizeof(dns_rr_withrottle)); // recycle most of buf
  _udp->write(buf, 10);

  _udp->write(_serviceName, _serviceName[0]+1);
  _udp->write(dns_rr_withrottle, sizeof(dns_rr_withrottle));
  // rr #3
  _udp->write(_serviceName, _serviceName[0]+1);
  _udp->write(dns_rr_withrottle, sizeof(dns_rr_withrottle));

  buf[1] = 0x21;                                  // recycle most of buf but here SRV
  buf[2] = 0x80;                                  // cache flush
  *((uint16_t*)(buf+8)) = lwip_htons(strlen(_serviceName) + strlen(dns_rr_local) + 1 + 6);
  _udp->write(buf, 10);

  byte srv[6];
  // priority and weight
  srv[0] = srv[1] = srv[2] = srv[3] = 0;
  // port
  *((uint16_t*)(srv+4)) = lwip_htons(_servicePort);
  _udp->write(srv, 6);
  // target
  _udp->write(_serviceName, _serviceName[0]+1);
  _udp->write(dns_rr_local, strlen(dns_rr_local)+1);

  // rr #4
  _udp->write(_serviceName, _serviceName[0]+1);
  _udp->write(dns_rr_local, strlen(dns_rr_local)+1);
  
  buf[1] = 0x01;                                  // recycle most of buf but here A
  *((uint16_t*)(buf+8)) = lwip_htons(4);
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
