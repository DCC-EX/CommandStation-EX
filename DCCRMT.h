/*
 *  © 2021, Harald Barth.
 *  
 *  This file is part of DCC-EX
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

#pragma once
#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "DCCPacket.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"

// make calculations easy and set up for microseconds
#define RMT_CLOCK_DIVIDER 80
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000

class RMTChannel {
 public:
  inline RMTChannel(byte pin, bool isMain) {
    if (isMain)
      RMTChannel(pin, 0, PREAMBLE_BITS_MAIN, 1);
    else
      RMTChannel(pin, 2, PREAMBLE_BITS_PROG, 0);
  };
  RMTChannel(byte pin, byte ch, byte plen, bool isProg);
  void IRAM_ATTR RMTinterrupt();
  void RMTprefill();
  bool RMTfillData(dccPacket packet);
  //bool RMTfillData(const byte buffer[], byte byteCount, byte repeatCount);
  
  static RMTChannel mainRMTChannel;
  static RMTChannel progRMTChannel;
  
 private:
    
  rmt_channel_t channel;
  // 3 types of data to send, preamble and then idle or data
  // if this is prog track, idle will contain reset instead
  rmt_item32_t *idle;
  byte idleLen;
  rmt_item32_t *preamble;
  byte preambleLen;
  rmt_item32_t *data;
  byte dataLen;
  byte maxDataLen;
  // flags 
  volatile bool preambleNext = true;  // alternate between preamble and content
  volatile bool dataReady = false;    // do we have real data available or send idle
  volatile byte dataRepeat = 0;
};
#endif //ESP32
