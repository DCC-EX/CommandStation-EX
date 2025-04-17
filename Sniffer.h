/*
 *  © 2025 Harald Barth
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
#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include <list>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"

#define MAXDCCPACKETLEN 8
#include "DCCPacket.h"

class Sniffer {
public:
  Sniffer(byte snifferpin);
  void IRAM_ATTR processInterrupt(int32_t capticks, bool posedge);
  inline int32_t getTicks() {
    noInterrupts();
    int32_t i = diffticks;
    interrupts();
    return i;
  };
  inline int64_t getDebug() {
    noInterrupts();
    int64_t i = debugfield;
    interrupts();
    return i;
  };
  inline DCCPacket fetchPacket() {
    // if there is no new data, this will create a
    // packet with length 0 (which is no packet)
    DCCPacket p;
    noInterrupts();
    if (!outpacket.empty()) {
      p = outpacket.front();
      outpacket.pop_front();
    }
    if (fetchflag) {
      fetchflag = false; // (data has been fetched)
    }
    interrupts();
    return p;
  };
  bool inputActive();
private:
  // keep these vars in processInterrupt only
  uint64_t bitfield = 0;
  uint64_t debugfield = 0;
  int32_t diffticks;
  int32_t lastticks;
  bool lastedge;
  byte currentbyte = 0;
  byte dccbytes[MAXDCCPACKETLEN];
  byte dcclen = 0;
  bool inpacket = false;
  // these vars are used as interface to other parts of sniffer
  byte halfbitcounter = 0;
  bool fetchflag = false;
  std::list<DCCPacket> outpacket;
  DCCPacket prevpacket;
  volatile unsigned long lastendofpacket = 0; // timestamp millis

};
#endif
