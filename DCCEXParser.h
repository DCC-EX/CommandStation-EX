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

typedef void (*FILTER_CALLBACK)(Print * stream, byte & opcode, byte & paramCount, int p[]);

struct DCCEXParser
{
   DCCEXParser();
   void loop(Stream & stream);
   void parse(Print * stream,  byte * command, bool blocking);
   void flush();
   static void setFilter(FILTER_CALLBACK filter);
   static const int MAX_PARAMS=10;  // Must not exceed this
 
   private:
  
    static const int MAX_BUFFER=50;  // longest command sent in
     byte  bufferLength=0;
     bool  inCommandPayload=false;
     bool  asyncBanned;   // true when called with stream that must complete before returning
     byte  buffer[MAX_BUFFER+2]; 
    int splitValues( int result[MAX_PARAMS], const byte * command);
     
     bool parseT(Print * stream, int params, int p[]);
     bool parseZ(Print * stream, int params, int p[]);
     bool parseS(Print * stream,  int params, int p[]);
     bool parsef(Print * stream,  int params, int p[]);

    
    static bool stashBusy;
   
    static Print * stashStream;
    static int stashP[MAX_PARAMS];
    bool stashCallback(Print * stream, int p[MAX_PARAMS]);
    static void callback_W(int result);
    static void callback_B(int result);        
    static void callback_R(int result);
    static FILTER_CALLBACK  filterCallback;
    static void funcmap(int cab, byte value, byte fstart, byte fstop);

};

#endif
