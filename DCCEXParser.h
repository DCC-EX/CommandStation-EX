/*
 *  © 2021 Mike S
 *  © 2021 Fred Decker
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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
#ifndef DCCEXParser_h
#define DCCEXParser_h
#include <Arduino.h>
#include "FSH.h"
#include "RingStream.h"
#include "defines.h"

typedef void (*FILTER_CALLBACK)(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
typedef void (*AT_COMMAND_CALLBACK)(HardwareSerial * stream,const byte * command);

struct DCCEXParser
{
   
   static void parse(Print * stream,  byte * command,  RingStream * ringStream);
   static void parse(const FSH * cmd);
   static void parseOne(Print * stream,  byte * command,  RingStream * ringStream);
   static void setFilter(FILTER_CALLBACK filter);
   static void setRMFTFilter(FILTER_CALLBACK filter);
   static void setAtCommandCallback(AT_COMMAND_CALLBACK filter);
   static const int MAX_COMMAND_PARAMS=10;  // Must not exceed this
 
   private:
  
    static const int16_t MAX_BUFFER=50;  // longest command sent in
    static int16_t splitValues( int16_t result[MAX_COMMAND_PARAMS], const byte * command, bool usehex);
     
    static bool parseT(Print * stream, int16_t params, int16_t p[]);
    static bool parseZ(Print * stream, int16_t params, int16_t p[]);
    static bool parseS(Print * stream, int16_t params, int16_t p[]);
    static bool parsef(Print * stream, int16_t params, int16_t p[]);
    static bool parseC(Print * stream, int16_t params, int16_t p[]);
    static bool parseD(Print * stream, int16_t params, int16_t p[]);
#ifndef IO_NO_HAL
    static bool parseI(Print * stream, int16_t params, int16_t p[]);
#endif

    static Print * getAsyncReplyStream();
    static void commitAsyncReplyStream();

    static bool stashBusy;
    static byte stashTarget;
    static Print * stashStream;
    static RingStream * stashRingStream;
    
    static int16_t stashP[MAX_COMMAND_PARAMS];
    static bool stashCallback(Print * stream, int16_t p[MAX_COMMAND_PARAMS], RingStream * ringStream);
    static void callback_W(int16_t result);
    static void callback_W4(int16_t result);
    static void callback_B(int16_t result);        
    static void callback_R(int16_t result);
    static void callback_Rloco(int16_t result);
    static void callback_Wloco(int16_t result);
    static void callback_Vbit(int16_t result);
    static void callback_Vbyte(int16_t result);
    static FILTER_CALLBACK  filterCallback;
    static FILTER_CALLBACK  filterRMFTCallback;
    static AT_COMMAND_CALLBACK  atCommandCallback;
    static bool funcmap(int16_t cab, byte value, byte fstart, byte fstop);
    static void sendFlashList(Print * stream,const int16_t flashList[]);

};

#endif
