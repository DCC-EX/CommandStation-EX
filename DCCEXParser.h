/*
 *  © 2020, Chris Harlow. All rights reserved.
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

typedef void (*FILTER_CALLBACK)(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
typedef void (*AT_COMMAND_CALLBACK)(const byte * command);

struct DCCEXParser
{
   DCCEXParser();
   void loop(Stream & stream);
   void parse(Print * stream,  byte * command,  RingStream * ringStream);
   void parse(const FSH * cmd);
   void flush();
   static void setFilter(FILTER_CALLBACK filter);
   static void setRMFTFilter(FILTER_CALLBACK filter);
   static void setAtCommandCallback(AT_COMMAND_CALLBACK filter);
   static const int MAX_COMMAND_PARAMS=10;  // Must not exceed this
 
   private:
  
    static const int16_t MAX_BUFFER=50;  // longest command sent in
     byte  bufferLength=0;
     bool  inCommandPayload=false;
     byte  buffer[MAX_BUFFER+2]; 
    int16_t splitValues( int16_t result[MAX_COMMAND_PARAMS], const byte * command);
    int16_t splitHexValues( int16_t result[MAX_COMMAND_PARAMS], const byte * command);
     
     bool parseT(Print * stream, int16_t params, int16_t p[]);
     bool parseZ(Print * stream, int16_t params, int16_t p[]);
     bool parseS(Print * stream,  int16_t params, int16_t p[]);
     bool parsef(Print * stream,  int16_t params, int16_t p[]);
     bool parseD(Print * stream,  int16_t params, int16_t p[]);

     static Print * getAsyncReplyStream();
     static void commitAsyncReplyStream();

    static bool stashBusy;
    static byte stashTarget;
    static Print * stashStream;
    static RingStream * stashRingStream;
    
    static int16_t stashP[MAX_COMMAND_PARAMS];
    bool stashCallback(Print * stream, int16_t p[MAX_COMMAND_PARAMS], RingStream * ringStream);
    static void callback_W(int16_t result);
    static void callback_B(int16_t result);        
    static void callback_R(int16_t result);
    static void callback_Rloco(int16_t result);
    static void callback_Wloco(int16_t result);
    static void callback_Vbit(int16_t result);
    static void callback_Vbyte(int16_t result);
    static FILTER_CALLBACK  filterCallback;
    static FILTER_CALLBACK  filterRMFTCallback;
    static AT_COMMAND_CALLBACK  atCommandCallback;
    static void funcmap(int16_t cab, byte value, byte fstart, byte fstop);

};

#endif
