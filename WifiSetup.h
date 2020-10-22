/*
 *  © 2020 Gregor Baues. All rights reserved.
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

#ifndef WifiSetup_h
#define WifiSetup_h

#include "NetworkSetup.h"
#include "WiFiEspAT.h"

// Emulate Serial1 on pins 6/7 if not present
#if defined(ARDUINO_ARCH_AVR) && !defined(HAVE_HWSERIAL1)
#include <SoftwareSerial.h>
SoftwareSerial Serial1(6, 7); // RX, TX
#define AT_BAUD_RATE 9600
#else
#define AT_BAUD_RATE 115200
#endif


class WifiSetup: public NetworkSetup {

private:

    WiFiServer*         server;
    WiFiUDP*            udp;

public:

    // WiFiServer *setup(uint16_t port);
    bool setup();

    WiFiUDP* getUdp() {
        return udp;
    }

    WiFiServer* getServer() {
        return server;
    }

    WifiSetup();
    WifiSetup(uint16_t port, protocolType protocol);
    ~WifiSetup();
};

#endif