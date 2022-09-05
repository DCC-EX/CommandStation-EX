/*
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2021 Chris Harlow
 *  © 2020 Gregor Baues
 *  All rights reserved.
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

#include "defines.h"
#include "DCCEXParser.h"
#include <Arduino.h>
#include <avr/pgmspace.h>
#if defined (ARDUINO_TEENSY41)
 #include <NativeEthernet.h>         //TEENSY Ethernet Treiber
 #include <NativeEthernetUdp.h>   
#else
 #include "Ethernet.h"
#endif
#include "RingStream.h"

/**
 * @brief Network Configuration
 * 
 */

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
  ~EthernetInterface();
  void loop2();
  bool checkLink();

  EthernetServer *server = nullptr;
  EthernetClient clients[MAX_SOCK_NUM]; // accept up to MAX_SOCK_NUM client connections at the same time
                                        // This depends on the chipset used on the Shield
  uint8_t buffer[MAX_ETH_BUFFER+1];     // buffer used by TCP for the recv
  RingStream * outboundRing = nullptr;
};

#endif
