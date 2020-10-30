/*
 *  © 2020,Gregor Baues,  Chris Harlow. All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
 *
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
 * 
 *  Ethernet Interface added by Gregor Baues
 */

#ifndef EthernetInterface_h
#define EthernetInterface_h

#include "DCCEXParser.h"
#include "MemStream.h"
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Ethernet.h>
#include "RingStream.h"

/* some generated mac addresses as EthernetShields don't have one by default in HW.
 * Sometimes they come on a sticker on the EthernetShield then use this address otherwise
 * just choose one from below or generate one yourself. Only condition is that there is no 
 * other device on your network with the same Mac address.
 * 
 * 52:b8:8a:8e:ce:21
 * e3:e9:73:e1:db:0d
 * 54:2b:13:52:ac:0c
 * c2:d8:d4:7d:7c:cb
 * 86:cf:fa:9f:07:79
 */

/**
 * @brief Network Configuration
 * 
 */
#define MAC_ADDRESS { 0x52, 0xB8, 0x8A, 0x8E, 0xCE, 0x21 }      // MAC address of your networking card found on the sticker on your card or take one from above
                                                                // this one is not used elsewhere and corresponds to your network layout
#define LISTEN_PORT 2560                                        // default listen port for the server 
#define MAX_ETH_BUFFER 512
#define OUTBOUND_RING_SIZE 2048

class EthernetInterface {

 public:
     
     static void setup();       
     static void loop();
   
 private:
     static EthernetInterface * singleton;
     bool connected;
     EthernetInterface();
     void loop2();
    EthernetServer * server;
    EthernetClient clients[MAX_SOCK_NUM];                // accept up to MAX_SOCK_NUM client connections at the same time; This depends on the chipset used on the Shield
    uint8_t buffer[MAX_ETH_BUFFER+1];                    // buffer used by TCP for the recv
    RingStream * outboundRing;
  
};

#endif
