// IP, ARP, UDP and TCP functions.
// Author: Guido Socher
// Copyright: GPL V2
//
// The TCP implementation uses some size optimisations which are valid
// only if all data can be sent in one single packet. This is however
// not a big limitation for a microcontroller as you will anyhow use
// small web-pages. The web server must send the entire web page in one
// packet. The client "web browser" as implemented here can also receive
// large pages.
//
// 2010-05-20 <jc@wippler.nl>

#include "EtherCard.h"
#include "net.h"
#undef word // arduino nonsense

#define gPB ether.buffer

#define PINGPATTERN 0x42

// Avoid spurious pgmspace warnings - http://forum.jeelabs.net/node/327
// See also http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
//#undef PROGMEM
//#define PROGMEM __attribute__(( section(".progmem.data") ))
//#undef PSTR
//#define PSTR(s) (__extension__({static prog_char c[] PROGMEM = (s); &c[0];}))

#define TCP_STATE_SENDSYN       1
#define TCP_STATE_SYNSENT       2
#define TCP_STATE_ESTABLISHED   3
#define TCP_STATE_NOTUSED       4
#define TCP_STATE_CLOSING       5
#define TCP_STATE_CLOSED        6

static uint8_t destmacaddr[ETH_LEN]; // storing both dns server and destination mac addresses, but at different times because both are never needed at same time.
static boolean waiting_for_dns_mac = false; //might be better to use bit flags and bitmask operations for these conditions
static boolean has_dns_mac = false;
static boolean waiting_for_dest_mac = false;
static boolean has_dest_mac = false;
static uint8_t gwmacaddr[ETH_LEN]; // Hardware (MAC) address of gateway router
static uint8_t waitgwmac; // Bitwise flags of gateway router status - see below for states
//Define gateway router ARP statuses
#define WGW_INITIAL_ARP 1 // First request, no answer yet
#define WGW_HAVE_GW_MAC 2 // Have gateway router MAC
#define WGW_REFRESHING 4 // Refreshing but already have gateway MAC
#define WGW_ACCEPT_ARP_REPLY 8 // Accept an ARP reply

const unsigned char arpreqhdr[] PROGMEM = { 0,1,8,0,6,4,0,1 }; // ARP request header
const unsigned char iphdr[] PROGMEM = { 0x45,0,0,0x82,0,0,0x40,0,0x20 }; //IP header
extern const uint8_t allOnes[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // Used for hardware (MAC) and IP broadcast addresses

static void fill_checksum(uint8_t dest, uint8_t off, uint16_t len,uint8_t type) {
    const uint8_t* ptr = gPB + off;
    uint32_t sum = type==1 ? IP_PROTO_UDP_V+len-8 :
                   type==2 ? IP_PROTO_TCP_V+len-8 : 0;
    while(len >1) {
        sum += (uint16_t) (((uint32_t)*ptr<<8)|*(ptr+1));
        ptr+=2;
        len-=2;
    }
    if (len)
        sum += ((uint32_t)*ptr)<<8;
    while (sum>>16)
        sum = (uint16_t) sum + (sum >> 16);
    uint16_t ck = ~ (uint16_t) sum;
    gPB[dest] = ck>>8;
    gPB[dest+1] = ck;
}

static void setMACs (const uint8_t *mac) {
    EtherCard::copyMac(gPB + ETH_DST_MAC, mac);
    EtherCard::copyMac(gPB + ETH_SRC_MAC, EtherCard::mymac);
}

static void setMACandIPs (const uint8_t *mac, const uint8_t *dst) {
    setMACs(mac);
    EtherCard::copyIp(gPB + IP_DST_P, dst);
    EtherCard::copyIp(gPB + IP_SRC_P, EtherCard::myip);
}

static boolean is_lan(const uint8_t source[IP_LEN], const uint8_t destination[IP_LEN]) {
    if(source[0] == 0 || destination[0] == 0) {
        return false;
    }
    for(int i = 0; i < IP_LEN; i++)
        if((source[i] & EtherCard::netmask[i]) != (destination[i] & EtherCard::netmask[i])) {
            return false;
        }
    return true;
}

static uint8_t eth_type_is_arp_and_my_ip(uint16_t len) {
    return len >= 41 && gPB[ETH_TYPE_H_P] == ETHTYPE_ARP_H_V &&
           gPB[ETH_TYPE_L_P] == ETHTYPE_ARP_L_V &&
           memcmp(gPB + ETH_ARP_DST_IP_P, EtherCard::myip, IP_LEN) == 0;
}

static uint8_t eth_type_is_ip_and_my_ip(uint16_t len) {
    return len >= 42 && gPB[ETH_TYPE_H_P] == ETHTYPE_IP_H_V &&
           gPB[ETH_TYPE_L_P] == ETHTYPE_IP_L_V &&
           gPB[IP_HEADER_LEN_VER_P] == 0x45 &&
           (memcmp(gPB + IP_DST_P, EtherCard::myip, IP_LEN) == 0  //not my IP
            || (memcmp(gPB + IP_DST_P, EtherCard::broadcastip, IP_LEN) == 0) //not subnet broadcast
            || (memcmp(gPB + IP_DST_P, allOnes, IP_LEN) == 0)); //not global broadcasts
    //!@todo Handle multicast
}

static void fill_ip_hdr_checksum() {
    gPB[IP_CHECKSUM_P] = 0;
    gPB[IP_CHECKSUM_P+1] = 0;
    gPB[IP_FLAGS_P] = 0x40; // don't fragment
    gPB[IP_FLAGS_P+1] = 0;  // fragment offset
    gPB[IP_TTL_P] = 64; // ttl
    fill_checksum(IP_CHECKSUM_P, IP_P, IP_HEADER_LEN,0);
}

static void make_eth_ip() {
    setMACs(gPB + ETH_SRC_MAC);
    EtherCard::copyIp(gPB + IP_DST_P, gPB + IP_SRC_P);
    EtherCard::copyIp(gPB + IP_SRC_P, EtherCard::myip);
    fill_ip_hdr_checksum();
}

static void make_arp_answer_from_request() {
    setMACs(gPB + ETH_SRC_MAC);
    gPB[ETH_ARP_OPCODE_H_P] = ETH_ARP_OPCODE_REPLY_H_V;
    gPB[ETH_ARP_OPCODE_L_P] = ETH_ARP_OPCODE_REPLY_L_V;
    EtherCard::copyMac(gPB + ETH_ARP_DST_MAC_P, gPB + ETH_ARP_SRC_MAC_P);
    EtherCard::copyMac(gPB + ETH_ARP_SRC_MAC_P, EtherCard::mymac);
    EtherCard::copyIp(gPB + ETH_ARP_DST_IP_P, gPB + ETH_ARP_SRC_IP_P);
    EtherCard::copyIp(gPB + ETH_ARP_SRC_IP_P, EtherCard::myip);
    EtherCard::packetSend(42);
}

static void make_echo_reply_from_request(uint16_t len) {
    make_eth_ip();
    gPB[ICMP_TYPE_P] = ICMP_TYPE_ECHOREPLY_V;
    if (gPB[ICMP_CHECKSUM_P] > (0xFF-0x08))
        gPB[ICMP_CHECKSUM_P+1]++;
    gPB[ICMP_CHECKSUM_P] += 0x08;
    EtherCard::packetSend(len);
}

void EtherCard::makeUdpReply (const char *data,uint8_t datalen,uint16_t port) {
    if (datalen>220)
        datalen = 220;
    gPB[IP_TOTLEN_H_P] = (IP_HEADER_LEN+UDP_HEADER_LEN+datalen) >>8;
    gPB[IP_TOTLEN_L_P] = IP_HEADER_LEN+UDP_HEADER_LEN+datalen;
    make_eth_ip();
    gPB[UDP_DST_PORT_H_P] = gPB[UDP_SRC_PORT_H_P];
    gPB[UDP_DST_PORT_L_P] = gPB[UDP_SRC_PORT_L_P];
    gPB[UDP_SRC_PORT_H_P] = port>>8;
    gPB[UDP_SRC_PORT_L_P] = port;
    gPB[UDP_LEN_H_P] = (UDP_HEADER_LEN+datalen) >> 8;
    gPB[UDP_LEN_L_P] = UDP_HEADER_LEN+datalen;
    gPB[UDP_CHECKSUM_H_P] = 0;
    gPB[UDP_CHECKSUM_L_P] = 0;
    memcpy(gPB + UDP_DATA_P, data, datalen);
    fill_checksum(UDP_CHECKSUM_H_P, IP_SRC_P, 16 + datalen,1);
    packetSend(UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN+datalen);
}

void EtherCard::udpPrepare (uint16_t sport, const uint8_t *dip, uint16_t dport) {
    if(is_lan(myip, dip)) {                    // this works because both dns mac and destinations mac are stored in same variable - destmacaddr
        setMACandIPs(destmacaddr, dip);        // at different times. The program could have separate variable for dns mac, then here should be
    } else {                                   // checked if dip is dns ip and separately if dip is hisip and then use correct mac.
        setMACandIPs(gwmacaddr, dip);
    }
    // see http://tldp.org/HOWTO/Multicast-HOWTO-2.html
    // multicast or broadcast address, https://github.com/njh/EtherCard/issues/59
    if ((dip[0] & 0xF0) == 0xE0 || *((unsigned long*) dip) == 0xFFFFFFFF || !memcmp(broadcastip,dip,IP_LEN))
        EtherCard::copyMac(gPB + ETH_DST_MAC, allOnes);
    gPB[ETH_TYPE_H_P] = ETHTYPE_IP_H_V;
    gPB[ETH_TYPE_L_P] = ETHTYPE_IP_L_V;
    memcpy_P(gPB + IP_P,iphdr,sizeof iphdr);
    gPB[IP_TOTLEN_H_P] = 0;
    gPB[IP_PROTO_P] = IP_PROTO_UDP_V;
    gPB[UDP_DST_PORT_H_P] = (dport>>8);
    gPB[UDP_DST_PORT_L_P] = dport;
    gPB[UDP_SRC_PORT_H_P] = (sport>>8);
    gPB[UDP_SRC_PORT_L_P] = sport;
    gPB[UDP_LEN_H_P] = 0;
    gPB[UDP_CHECKSUM_H_P] = 0;
    gPB[UDP_CHECKSUM_L_P] = 0;
}

void EtherCard::udpTransmit (uint16_t datalen) {
    gPB[IP_TOTLEN_H_P] = (IP_HEADER_LEN+UDP_HEADER_LEN+datalen) >> 8;
    gPB[IP_TOTLEN_L_P] = IP_HEADER_LEN+UDP_HEADER_LEN+datalen;
    fill_ip_hdr_checksum();
    gPB[UDP_LEN_H_P] = (UDP_HEADER_LEN+datalen) >>8;
    gPB[UDP_LEN_L_P] = UDP_HEADER_LEN+datalen;
    fill_checksum(UDP_CHECKSUM_H_P, IP_SRC_P, 16 + datalen,1);
    packetSend(UDP_HEADER_LEN+IP_HEADER_LEN+ETH_HEADER_LEN+datalen);
}

void EtherCard::sendUdp (const char *data, uint8_t datalen, uint16_t sport,
                         const uint8_t *dip, uint16_t dport) {
    udpPrepare(sport, dip, dport);
    if (datalen>220)
        datalen = 220;
    memcpy(gPB + UDP_DATA_P, data, datalen);
    udpTransmit(datalen);
}

// make a arp request
static void client_arp_whohas(uint8_t *ip_we_search) {
    setMACs(allOnes);
    gPB[ETH_TYPE_H_P] = ETHTYPE_ARP_H_V;
    gPB[ETH_TYPE_L_P] = ETHTYPE_ARP_L_V;
    memcpy_P(gPB + ETH_ARP_P, arpreqhdr, sizeof arpreqhdr);
    memset(gPB + ETH_ARP_DST_MAC_P, 0, ETH_LEN);
    EtherCard::copyMac(gPB + ETH_ARP_SRC_MAC_P, EtherCard::mymac);
    EtherCard::copyIp(gPB + ETH_ARP_DST_IP_P, ip_we_search);
    EtherCard::copyIp(gPB + ETH_ARP_SRC_IP_P, EtherCard::myip);
    EtherCard::packetSend(42);
}

uint8_t EtherCard::clientWaitingGw () {
    return !(waitgwmac & WGW_HAVE_GW_MAC);
}

static uint8_t client_store_mac(uint8_t *source_ip, uint8_t *mac) {
    if (memcmp(gPB + ETH_ARP_SRC_IP_P, source_ip, IP_LEN) != 0)
        return 0;
    EtherCard::copyMac(mac, gPB + ETH_ARP_SRC_MAC_P);
    return 1;
}

// static void client_gw_arp_refresh() {
//   if (waitgwmac & WGW_HAVE_GW_MAC)
//     waitgwmac |= WGW_REFRESHING;
// }

void EtherCard::setGwIp (const uint8_t *gwipaddr) {
    delaycnt = 0; //request gateway ARP lookup
    waitgwmac = WGW_INITIAL_ARP; // causes an arp request in the packet loop
    copyIp(gwip, gwipaddr);
}

void EtherCard::updateBroadcastAddress()
{
    for(uint8_t i=0; i<IP_LEN; i++)
        broadcastip[i] = myip[i] | ~netmask[i];
}

uint16_t EtherCard::packetLoop (uint16_t plen) {

    if (plen==0) {
        //Check every 65536 (no-packet) cycles whether we need to retry ARP request for gateway
        if ((waitgwmac & WGW_INITIAL_ARP || waitgwmac & WGW_REFRESHING) &&
                delaycnt==0 && isLinkUp()) {
            client_arp_whohas(gwip);
            waitgwmac |= WGW_ACCEPT_ARP_REPLY;
        }
        delaycnt++;

        //!@todo this is trying to find mac only once. Need some timeout to make another call if first one doesn't succeed.
        if(is_lan(myip, dnsip) && !has_dns_mac && !waiting_for_dns_mac) {
            client_arp_whohas(dnsip);
            waiting_for_dns_mac = true;
        }

        //!@todo this is trying to find mac only once. Need some timeout to make another call if first one doesn't succeed.
        if(is_lan(myip, hisip) && !has_dest_mac && !waiting_for_dest_mac) {
            client_arp_whohas(hisip);
            waiting_for_dest_mac = true;
        }

        return 0;
    }

    if (eth_type_is_arp_and_my_ip(plen))
    {   //Service ARP request
        if (gPB[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REQ_L_V)
            make_arp_answer_from_request();
        if (waitgwmac & WGW_ACCEPT_ARP_REPLY && (gPB[ETH_ARP_OPCODE_L_P]==ETH_ARP_OPCODE_REPLY_L_V) && client_store_mac(gwip, gwmacaddr))
            waitgwmac = WGW_HAVE_GW_MAC;
        if (!has_dns_mac && waiting_for_dns_mac && client_store_mac(dnsip, destmacaddr)) {
            has_dns_mac = true;
            waiting_for_dns_mac = false;
        }
        if (!has_dest_mac && waiting_for_dest_mac && client_store_mac(hisip, destmacaddr)) {
            has_dest_mac = true;
            waiting_for_dest_mac = false;
        }
        return 0;
    }

    if (eth_type_is_ip_and_my_ip(plen)==0)
    {   //Not IP so ignoring
        //!@todo Add other protocols (and make each optional at compile time)
        return 0;
    }

#if ETHERCARD_ICMP
    if (gPB[IP_PROTO_P]==IP_PROTO_ICMP_V && gPB[ICMP_TYPE_P]==ICMP_TYPE_ECHOREQUEST_V)
    {   //Service ICMP echo request (ping)
        make_echo_reply_from_request(plen);
        return 0;
    }
#endif

    if (plen < UDP_DATA_P || gPB[IP_PROTO_P]!=IP_PROTO_UDP_V )
        return 0; //from here on we are only interested in UDP-packets

    return plen + UDP_DATA_P;  // Offset of UDP payload.
}

void EtherCard::copyIp (uint8_t *dst, const uint8_t *src) {
    memcpy(dst, src, IP_LEN);
}

void EtherCard::copyMac (uint8_t *dst, const uint8_t *src) {
    memcpy(dst, src, ETH_LEN);
}
