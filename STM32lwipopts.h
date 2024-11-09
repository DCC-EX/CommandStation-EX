/**
  ******************************************************************************
  * @file    STM32lwipopts_default.h
  * @author  MCD Application Team
  * @brief   lwIP Options Configuration.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#ifndef __STM32LWIPOPTS_H__
#define __STM32LWIPOPTS_H__

// we can not include that here so we
// need to duplicate that.define
// #include "defines.h"
#define MAX_NUM_TCP_CLIENTS 20

// increase ARP cache
#define MEMP_NUM_ARP_QUEUE MAX_NUM_TCP_CLIENTS+3 // one for each client (all on different HW) and a few extra

// Example for debug
//#define LWIP_DEBUG                      1
//#define TCP_DEBUG                       LWIP_DBG_ON

/**
 * NO_SYS==1: Provides VERY minimal functionality. Otherwise,
 * use lwIP facilities.
 */
#define NO_SYS                  1

/**
 * SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection for certain
 * critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT    0

#define LWIP_NOASSERT

/* ---------- Memory options ---------- */
/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           4

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE                (10*1024)

// Could be better or worse, needs more tests
//#define MEM_LIBC_MALLOC         1
//#define MEMP_MEM_MALLOC         1

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF           10
/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
   per active UDP "connection". */
#define MEMP_NUM_UDP_PCB        6
/* MEMP_NUM_TCP_PCB: the number of simulatenously active TCP
   connections. */
#define MEMP_NUM_TCP_PCB        MAX_NUM_TCP_CLIENTS+1 // one extra so we can reject number N+1 from our code
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
#define MEMP_NUM_TCP_PCB_LISTEN 6
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
#define MEMP_NUM_TCP_SEG        MAX_NUM_TCP_CLIENTS
/* MEMP_NUM_SYS_TIMEOUT: the number of simulateously active
   timeouts. */
#define MEMP_NUM_SYS_TIMEOUT    MAX_NUM_TCP_CLIENTS+2


/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          MAX_NUM_TCP_CLIENTS

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */
#define PBUF_POOL_BUFSIZE       1524


/* ---------- TCP options ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255
#define LWIP_SO_RCVTIMEO                1
#define LWIP_SO_RCVRCVTIMEO_NONSTANDARD 1   /* Pass an integer number of ms instead of a timeval struct. */
#define LWIP_SO_SNDTIMEO                1
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 1   /* Pass an integer number of ms instead of a timeval struct. */

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory and you are not scared by TCP congestion and latencies. */
#define TCP_QUEUE_OOSEQ         0

/* TCP Maximum segment size. */
#define TCP_MSS                 (1500 - 40)   /* TCP_MSS = (Ethernet MTU - IP header size - TCP header size) */

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             (4*TCP_MSS)

/*  TCP_SND_QUEUELEN: TCP sender buffer space (pbufs). This must be at least
  as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work. */

#define TCP_SND_QUEUELEN        (2* TCP_SND_BUF/TCP_MSS)

/* TCP receive window. */
#define TCP_WND                 (3*TCP_MSS)

#define LWIP_TCP_KEEPALIVE                  1   /* Keep the TCP link active. Important for MQTT/TLS */
#define LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS  1   /* Prevent the same port to be used after reset.
                                                   Otherwise, the remote host may be confused if the port was not explicitly closed before the reset. */


/* ---------- ICMP options ---------- */
#define LWIP_ICMP                       1
#define LWIP_RAW                        1 /* PING changed to 1 */
#define DEFAULT_RAW_RECVMBOX_SIZE       3 /* for ICMP PING */


/* ---------- DHCP options ---------- */
/* Define LWIP_DHCP to 1 if you want DHCP configuration of
   interfaces. DHCP is not implemented in lwIP 0.5.1, however, so
   turning this on does currently not work. */
#define LWIP_DHCP               1


/* ---------- UDP options ---------- */
#define LWIP_UDP                1
#define UDP_TTL                 255


/* ---------- Statistics options ---------- */
#define LWIP_STATS 0
#define LWIP_PROVIDE_ERRNO

/* ---------- link callback options ---------- */
/* LWIP_NETIF_LINK_CALLBACK==1: Support a callback function from an interface
 * whenever the link changes (i.e., link down)
 */
// need for building net_ip.c
#define LWIP_NETIF_HOSTNAME 1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_DHCP_CHECK_LINK_UP         1

/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/

/*
The STM32F4x7 allows computing and verifying the IP, UDP, TCP and ICMP checksums by hardware:
 - To use this feature let the following define uncommented.
 - To disable it and process by CPU comment the  the checksum.
*/
#define CHECKSUM_BY_HARDWARE


#ifdef CHECKSUM_BY_HARDWARE
  /* CHECKSUM_GEN_IP==0: Generate checksums by hardware for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 0
  /* CHECKSUM_GEN_UDP==0: Generate checksums by hardware for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                0
  /* CHECKSUM_GEN_TCP==0: Generate checksums by hardware for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                0
  /* CHECKSUM_CHECK_IP==0: Check checksums by hardware for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               0
  /* CHECKSUM_CHECK_UDP==0: Check checksums by hardware for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              0
  /* CHECKSUM_CHECK_TCP==0: Check checksums by hardware for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              0
  /* CHECKSUM_CHECK_ICMP==0: Check checksums by hardware for incoming ICMP packets.*/
  #define CHECKSUM_GEN_ICMP               0
#else
  /* CHECKSUM_GEN_IP==1: Generate checksums in software for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 1
  /* CHECKSUM_GEN_UDP==1: Generate checksums in software for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                1
  /* CHECKSUM_GEN_TCP==1: Generate checksums in software for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                1
  /* CHECKSUM_CHECK_IP==1: Check checksums in software for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               1
  /* CHECKSUM_CHECK_UDP==1: Check checksums in software for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              1
  /* CHECKSUM_CHECK_TCP==1: Check checksums in software for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              1
  /* CHECKSUM_CHECK_ICMP==1: Check checksums by hardware for incoming ICMP packets.*/
  #define CHECKSUM_GEN_ICMP               1
#endif


/*
   ----------------------------------------------
   ---------- Sequential layer options ----------
   ----------------------------------------------
*/
/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN                    0

/*
   ------------------------------------
   ---------- Socket options ----------
   ------------------------------------
*/
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET                     0
#define LWIP_DNS                        1

/*
   ------------------------------------
   ---------- httpd options ----------
   ------------------------------------
*/

/** Set this to 1 to support CGI */
#define LWIP_HTTPD_CGI            1

/** Set this to 1 to support SSI (Server-Side-Includes) */
#define LWIP_HTTPD_SSI            1

/** Set this to 1 to include "fsdata_custom.c" instead of "fsdata.c" for the
 * file system (to prevent changing the file included in CVS) */
#define HTTPD_USE_CUSTOM_FSDATA   1

/*
   ------------------------------------
   ---------- Custom options ----------
   ------------------------------------
*/

/** Enables the Ethernet peripheral in RMII mode. If not defined, MII mode will
    be enabled. Pin mapping must be configured for the selected mode
    (see PinMap_Ethernet in PeripheralPins.c). */
#define ETHERNET_RMII_MODE_CONFIGURATION 1

/** Uncomment this line to use the ethernet input in interrupt mode.
  * NOTE: LwIP stack documentation recommends to use the polling mode without
  * an operating system. */
//#define ETH_INPUT_USE_IT 1

// We don't need this as we do not need to respond - we only announce
//#define LWIP_MDNS_RESPONDER             1
//#define LWIP_NUM_NETIF_CLIENT_DATA      1 // MDNS needs at least one
#define LWIP_IGMP                       1
#define SO_REUSE 1
#define SO_REUSE_RXTOALL 1
#warning testing this

#endif /* __STM32LWIPOPTS_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
