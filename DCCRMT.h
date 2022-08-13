/*
 *  © 2021-2022, Harald Barth.
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

#if defined(ARDUINO_ARCH_ESP32)
#pragma once
#include <Arduino.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"
#include "MotorDriver.h" // for class pinpair

// make calculations easy and set up for microseconds
#define RMT_CLOCK_DIVIDER 80
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000

class RMTChannel {
 public:
  RMTChannel(pinpair pins, bool isMain);
  bool addPin(byte pin, bool inverted=0);
  bool addPin(pinpair pins);
  void IRAM_ATTR RMTinterrupt();
  void RMTprefill();
  //int RMTfillData(dccPacket packet);
  int RMTfillData(const byte buffer[], byte byteCount, byte repeatCount);
  inline bool busy() {
    if (dataRepeat > 0) // we have still old work to do
      return true;
    return dataReady;
  };
  inline uint32_t packetCount() { return packetCounter; };
  
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
  uint32_t packetCounter = 0;
  // flags 
  volatile bool dataReady = false;    // do we have real data available or send idle
  volatile byte dataRepeat = 0;
};
#endif //ESP32
