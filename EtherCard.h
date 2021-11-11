// This code slightly follows the conventions of, but is not derived from:
//      EHTERSHIELD_H library for Arduino etherShield
//      Copyright (c) 2008 Xing Yu.  All right reserved. (this is LGPL v2.1)
// It is however derived from the enc28j60 and ip code (which is GPL v2)
//      Author: Pascal Stang
//      Modified by: Guido Socher
//      DHCP code: Andrew Lindsay
// Hence: GPL V2
//
// 2010-05-19 <jc@wippler.nl>
//
//
// PIN Connections (Using Arduino UNO):
//   VCC -   3.3V
//   GND -    GND
//   SCK - Pin 13
//   SO  - Pin 12
//   SI  - Pin 11
//   CS  - Pin  8
//
/** @file */

#ifndef EtherCard_h
#define EtherCard_h
// #ifndef __PROG_TYPES_COMPAT__
//   #define __PROG_TYPES_COMPAT__
// #endif

// #if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
// #define WRITE_RESULT size_t
// #define WRITE_RETURN return 1;
// #else
// #include <WProgram.h> // Arduino 0022
// #define WRITE_RESULT void
// #define WRITE_RETURN
// #endif

// #include <avr/pgmspace.h>
#include "enc28j60.h"

// Based on the net.h file from the AVRlib library by Pascal Stang.
// Author: Guido Socher
// Copyright: GPL V2
//
// For AVRlib See http://www.procyonengineering.com/
// Used with explicit permission of Pascal Stang.
//
// 2010-05-20 <jc@wippler.nl>

// notation: _P = position of a field
//           _V = value of a field

// ******* ETH *******
#define ETH_HEADER_LEN    14
#define ETH_LEN 6
// values of certain bytes:
#define ETHTYPE_ARP_H_V 0x08
#define ETHTYPE_ARP_L_V 0x06
#define ETHTYPE_IP_H_V  0x08
#define ETHTYPE_IP_L_V  0x00
// byte positions in the ethernet frame:
//
// Ethernet type field (2bytes):
#define ETH_TYPE_H_P 12
#define ETH_TYPE_L_P 13
//
#define ETH_DST_MAC 0
#define ETH_SRC_MAC 6


// ******* ARP *******
#define ETH_ARP_OPCODE_REPLY_H_V 0x0
#define ETH_ARP_OPCODE_REPLY_L_V 0x02
#define ETH_ARP_OPCODE_REQ_H_V 0x0
#define ETH_ARP_OPCODE_REQ_L_V 0x01
// start of arp header:
#define ETH_ARP_P 0xe
//
#define ETHTYPE_ARP_L_V 0x06
// arp.dst.ip
#define ETH_ARP_DST_IP_P 0x26
// arp.opcode
#define ETH_ARP_OPCODE_H_P 0x14
#define ETH_ARP_OPCODE_L_P 0x15
// arp.src.mac
#define ETH_ARP_SRC_MAC_P 0x16
#define ETH_ARP_SRC_IP_P 0x1c
#define ETH_ARP_DST_MAC_P 0x20
#define ETH_ARP_DST_IP_P 0x26

// ******* IP *******
#define IP_HEADER_LEN    20
#define IP_LEN 4
// ip.src
#define IP_SRC_P 0x1a
#define IP_DST_P 0x1e
#define IP_HEADER_LEN_VER_P 0xe
#define IP_CHECKSUM_P 0x18
#define IP_TTL_P 0x16
#define IP_FLAGS_P 0x14
#define IP_P 0xe
#define IP_TOTLEN_H_P 0x10
#define IP_TOTLEN_L_P 0x11

#define IP_PROTO_P 0x17

#define IP_PROTO_ICMP_V 1
#define IP_PROTO_TCP_V 6
// 17=0x11
#define IP_PROTO_UDP_V 17
// ******* ICMP *******
#define ICMP_TYPE_ECHOREPLY_V 0
#define ICMP_TYPE_ECHOREQUEST_V 8
//
#define ICMP_TYPE_P 0x22
#define ICMP_CHECKSUM_P 0x24
#define ICMP_CHECKSUM_H_P 0x24
#define ICMP_CHECKSUM_L_P 0x25
#define ICMP_IDENT_H_P 0x26
#define ICMP_IDENT_L_P 0x27
#define ICMP_DATA_P 0x2a

// ******* UDP *******
#define UDP_HEADER_LEN    8
//
#define UDP_SRC_PORT_H_P 0x22
#define UDP_SRC_PORT_L_P 0x23
#define UDP_DST_PORT_H_P 0x24
#define UDP_DST_PORT_L_P 0x25
//
#define UDP_LEN_H_P 0x26
#define UDP_LEN_L_P 0x27
#define UDP_CHECKSUM_H_P 0x28
#define UDP_CHECKSUM_L_P 0x29
#define UDP_DATA_P 0x2a


/** Enable DHCP.
*   Setting this to zero disables the use of DHCP; if a program uses DHCP it will
*   still compile but the program will not work. Saves about 60 bytes SRAM and
*   1550 bytes flash.
*/
#define ETHERCARD_DHCP 0

/** Enable client connections.
* Setting this to zero means that the program cannot issue TCP client requests
* anymore. Compilation will still work but the request will never be
* issued. Saves 4 bytes SRAM and 550 byte flash.
*/
#define ETHERCARD_TCPCLIENT 0

/** Enable TCP server functionality.
*   Setting this to zero means that the program will not accept TCP client
*   requests. Saves 2 bytes SRAM and 250 bytes flash.
*/
#define ETHERCARD_TCPSERVER 0

/** Enable UDP server functionality.
*   If zero UDP server is disabled. It is
*   still possible to register callbacks but these will never be called. Saves
*   about 40 bytes SRAM and 200 bytes flash. If building with -flto this does not
*   seem to save anything; maybe the linker is then smart enough to optimize the
*   call away.
*/
#define ETHERCARD_UDPSERVER 0

/** Enable automatic reply to pings.
*   Setting to zero means that the program will not automatically answer to
*   PINGs anymore. Also the callback that can be registered to answer incoming
*   pings will not be called. Saves 2 bytes SRAM and 230 bytes flash.
*/
#define ETHERCARD_ICMP 1

/** Enable use of stash.
*   Setting this to zero means that the stash mechanism cannot be used. Again
*   compilation will still work but the program may behave very unexpectedly.
*   Saves 30 bytes SRAM and 80 bytes flash.
*/
#define ETHERCARD_STASH 0


/** This type definition defines the structure of a UDP server event handler callback function */
typedef void (*UdpServerCallback)(
    uint16_t dest_port,    ///< Port the packet was sent to
    uint8_t src_ip[IP_LEN],    ///< IP address of the sender
    uint16_t src_port,    ///< Port the packet was sent from
    const char *data,   ///< UDP payload data
    uint16_t len);        ///< Length of the payload data

/** This type definition defines the structure of a DHCP Option callback function */
typedef void (*DhcpOptionCallback)(
    uint8_t option,     ///< The option number
    const byte* data,   ///< DHCP option data
    uint8_t len);       ///< Length of the DHCP option data



/** This class provides the main interface to a ENC28J60 based network interface card and is the class most users will use.
*   @note   All TCP/IP client (outgoing) connections are made from source port in range 2816-3071. Do not use these source ports for other purposes.
*/
class EtherCard : public ENC28J60 {
public:
    static uint8_t mymac[ETH_LEN];  ///< MAC address
    static uint8_t myip[IP_LEN];    ///< IP address
    static uint8_t netmask[IP_LEN]; ///< Netmask
    static uint8_t broadcastip[IP_LEN]; ///< Subnet broadcast address
    static uint8_t gwip[IP_LEN];   ///< Gateway
    static uint8_t dhcpip[IP_LEN]; ///< DHCP server IP address
    static uint8_t dnsip[IP_LEN];  ///< DNS server IP address
    static uint8_t hisip[IP_LEN];  ///< DNS lookup result
    static uint16_t hisport;  ///< TCP port to connect to (default 80)
    static bool using_dhcp;   ///< True if using DHCP
    static bool persist_tcp_connection; ///< False to break connections on first packet received
    static uint16_t delaycnt; ///< Counts number of cycles of packetLoop when no packet received - used to trigger periodic gateway ARP request

    static uint8_t begin (const uint16_t size, const uint8_t* macaddr,
                          uint8_t csPin = SS);
    static bool staticSetup (const uint8_t* my_ip,
                             const uint8_t* gw_ip = 0,
                             const uint8_t* dns_ip = 0,
                             const uint8_t* mask = 0);
    static void makeUdpReply (const char *data, uint8_t len, uint16_t port);
    static uint16_t packetLoop (uint16_t plen);
    static void setGwIp (const uint8_t *gwipaddr);
    static void updateBroadcastAddress();
    static uint8_t clientWaitingGw ();
    static void udpPrepare (uint16_t sport, const uint8_t *dip, uint16_t dport);
    static void udpTransmit (uint16_t len);
    static void sendUdp (const char *data, uint8_t len, uint16_t sport,
                         const uint8_t *dip, uint16_t dport);
    static void copyIp (uint8_t *dst, const uint8_t *src);
    static void copyMac (uint8_t *dst, const uint8_t *src);
};

extern EtherCard ether; //!< Global presentation of EtherCard class

#endif
