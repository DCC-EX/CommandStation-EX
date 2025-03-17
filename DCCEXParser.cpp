/*
 *  © 2022 Paul M Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021-2025 Herb Morton
 *  © 2020-2023 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2021 Fred Decker
 *  © 2020-2025 Chris Harlow
 *  © 2022 Colin Murdoch
 *  All rights reserved.
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

/*
List of single character OPCODEs in use for reference.

When determining a new OPCODE for a new feature, refer to this list as the source of truth.

Once a new OPCODE is decided upon, update this list.

  Character, Usage
  /, |EX-R| interactive commands
  -, Remove from reminder table
  =, |TM| configuration
  !, Emergency stop
  @, Reserved for future use - LCD messages to JMRI
  #, Request number of supported cabs/locos; heartbeat
  +, WiFi AT commands
  ?, Reserved for future use
  0, Track power off
  1, Track power on
  a, DCC accessory control
  A, DCC extended accessory control
  b, Write CV bit on main
  B, Write CV bit
  c, Request current command
  C, configure the CS
  d,
  D, Diagnostic commands
  e, Erase EEPROM
  E, Store configuration in EEPROM
  f, Loco decoder function control (deprecated)
  F, Loco decoder function control
  g,
  G,
  h,
  H, Turnout state broadcast
  i, Server details string
  I, Turntable object command, control, and broadcast
  j, Throttle responses
  J, Throttle queries
  k, Block exit  (Railcom)
  K, Block enter (Railcom)
  l, Loco speedbyte/function map broadcast
  L, Reserved for LCC interface (implemented in EXRAIL)
  m, message to throttles (broadcast output) 
  m, set momentum  
  M, Write DCC packet
  n, Reserved for SensorCam
  N, Reserved for Sensorcam 
  o, Neopixel driver (see also IO_NeoPixel.h)
  O, Output broadcast
  p, Broadcast power state
  P, Write DCC packet
  q, Sensor deactivated
  Q, Sensor activated
  r, Broadcast address read on programming track
  R, Read CVs
  s, Display status
  S, Sensor configuration
  t, Cab/loco update command
  T, Turnout configuration/control
  u, Reserved for user commands
  U, Reserved for user commands
  v,
  V, Verify CVs
  w, Write CV on main
  W, Write CV
  x,
  X, Invalid command response
  y, 
  Y, Output broadcast
  z, Direct output
  Z, Output configuration/control
*/

#include "StringFormatter.h"
#include "DCCEXParser.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "Turnouts.h"
#include "Outputs.h"
#include "Sensors.h"
#include "GITHUB_SHA.h"
#include "version.h"
#include "defines.h"
#include "CommandDistributor.h"
#include "EEStore.h"
#include "DIAG.h"
#include "TrackManager.h"
#include "DCCTimer.h"
#include "EXRAIL2.h"
#include "Turntables.h"
#include "version.h"
#include "KeywordHasher.h"
#include "CamParser.h"
#include "Stash.h"
#include "DCCEXParserMacros.h"

#ifdef ARDUINO_ARCH_ESP32
#include "WifiESP32.h"
#endif

// This macro can't be created easily as a portable function because the
// flashlist requires a far pointer for high flash access. 
#define SENDFLASHLIST(stream,flashList)                 \
    for (int16_t i=0;;i+=sizeof(flashList[0])) {                            \
        int16_t value=GETHIGHFLASHW(flashList,i);       \
        if (value==INT16_MAX) break;                            \
        StringFormatter::send(stream,F(" %d"),value);	\
    }                                   

int16_t DCCEXParser::stashP[MAX_COMMAND_PARAMS];
bool DCCEXParser::stashBusy;
Print *DCCEXParser::stashStream = NULL;
RingStream *DCCEXParser::stashRingStream = NULL;
byte DCCEXParser::stashTarget=0;

// This is a DCC-EX command parser.
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
// Non-DCC things like turnouts, pins and sensors are handled in additional interface classes.


int16_t DCCEXParser::splitValues(int16_t result[MAX_COMMAND_PARAMS], byte *cmd, bool usehex)
{
    byte state = 1;
    byte parameterCount = 0;
    int16_t runningValue = 0;
    byte *remainingCmd = cmd + 1; // skips the opcode
    bool signNegative = false;

    // clear all parameters in case not enough found
    for (int16_t i = 0; i < MAX_COMMAND_PARAMS; i++)
        result[i] = 0;

    while (parameterCount < MAX_COMMAND_PARAMS)
    {
        byte hot = *remainingCmd;
        switch (state)
        {

        case 1: // skipping spaces before a param
            if (hot == ' ')
                break;
            if (hot == '\0')
	      return -1;
	    if (hot == '>')
	      return parameterCount;
            state = 2;
            continue;

        case 2: // checking sign or quoted string
	    if (hot == '"') {
	      // this inserts an extra parameter 0x7777 in front
	      // of each string parameter as a marker that can
	      // be checked that a string parameter follows
	      // This clashes of course with the real value
	      // 0x7777 which we hope is used seldom
	      result[parameterCount] = (int16_t)0x7777;
	      parameterCount++;
	      result[parameterCount] = (int16_t)(remainingCmd - cmd + 1);
	      parameterCount++;
	      state = 4;
	      break;
	    }
            signNegative = false;
            runningValue = 0;
            state = 3;
            if (hot != '-')
                continue;
            signNegative = true;
            break;
        case 3: // building a parameter
            if (hot >= '0' && hot <= '9')
            {
                runningValue = (usehex?16:10) * runningValue + (hot - '0');
                break;
            }
            if (hot >= 'a' && hot <= 'z') hot=hot-'a'+'A'; // uppercase a..z
            if (usehex && hot>='A' && hot<='F') {
                // treat A..F as hex not keyword
                runningValue = 16 * runningValue + (hot - 'A' + 10);
                break;
            }
            if (hot=='_' || (hot >= 'A' && hot <= 'Z'))
            {
                // Super Kluge to turn keywords into a hash value that can be recognised later
                runningValue = ((runningValue << 5) + runningValue) ^ hot;
                break;
            }
            result[parameterCount] = runningValue * (signNegative ? -1 : 1);
            parameterCount++;
            state = 1;
            continue;
	case 4: // skipover text
	  if (hot == '\0')        // We did run to end of buffer without finding the "
	    return -1;
	  if (hot == '"') {
	    *remainingCmd = '\0'; // overwrite " in command buffer with the end-of-string
	    state = 1;
	  }
	  break;
        }
        remainingCmd++;
    }
    return parameterCount;
}

extern __attribute__((weak))  void myFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
FILTER_CALLBACK DCCEXParser::filterCallback = myFilter;
FILTER_CALLBACK DCCEXParser::filterRMFTCallback = 0;
FILTER_CALLBACK DCCEXParser::filterCamParserCallback = 0;
AT_COMMAND_CALLBACK DCCEXParser::atCommandCallback = 0;

// deprecated
void DCCEXParser::setFilter(FILTER_CALLBACK filter)
{
    filterCallback = filter;
}
void DCCEXParser::setRMFTFilter(FILTER_CALLBACK filter)
{
    filterRMFTCallback = filter;
}
void DCCEXParser::setCamParserFilter(FILTER_CALLBACK filter)
{
    filterCamParserCallback = filter;
}
void DCCEXParser::setAtCommandCallback(AT_COMMAND_CALLBACK callback)
{
    atCommandCallback = callback;
}

// Parse an F() string 
void DCCEXParser::parse(const FSH * cmd) {
      DIAG(F("SETUP(\"%S\")"),cmd);
      int size=STRLEN_P((char *)cmd)+1; 
      char buffer[size];
      STRCPY_P(buffer,(char *)cmd);
      parse(&USB_SERIAL,(byte *)buffer,NULL);
}

// See documentation on DCC class for info on this section

void DCCEXParser::parse(Print *stream,  byte *com,  RingStream *ringStream) {
  // This function can get stings of the form "<C OMM AND>" or "C OMM AND"
  // found is true first after the leading "<" has been passed
  bool found = (com[0] != '<');
  for (byte *c=com; c[0] != '\0'; c++) {
    if (found) {
      parseOne(stream, c,  ringStream);
      found=false;
    }
    if (c[0] == '<')
      found = true;
  }
}

void DCCEXParser::parseOne(Print *stream, byte *com, RingStream * ringStream)
{
#ifdef DISABLE_PROG
    (void)ringStream;
#endif
#ifndef DISABLE_EEPROM
    (void)EEPROM; // tell compiler not to warn this is unused
#endif
    byte params = 0;
    if (Diag::CMD)
        DIAG(F("PARSING:%s"), com);
    int16_t p[MAX_COMMAND_PARAMS];
    while (com[0] == '<' || com[0] == ' ')
        com++; // strip off any number of < or spaces
    byte opcode = com[0];

    if (opcode=='+') {
        if (atCommandCallback && !ringStream) {
          TrackManager::setPower(POWERMODE::OFF);
          atCommandCallback((HardwareSerial *)stream,com);
        }
        return; // we cant parse the <+ wifistuff > here
    }

    int16_t splitnum = splitValues(p, com, opcode=='M' || opcode=='P');
    if (splitnum<0 || splitnum>=MAX_COMMAND_PARAMS) {
        DIAG(F("Too many parameters"));
        return;
    }
    
    // Because of check above we are now inside byte size
    params = splitnum;

    if (filterCallback)
        filterCallback(stream, opcode, params, p);
    if (filterRMFTCallback && opcode!='\0')
        filterRMFTCallback(stream, opcode, params, p);
    if (filterCamParserCallback && opcode!='\0')
        filterCamParserCallback(stream, opcode, params, p);
    if (opcode=='\0') return; // filterCallback asked us to ignore
    
    
    matchedCommandFormat = F("none");
    checkFailedFormat = matchedCommandFormat;
    if (execute(com,stream, opcode, params, p, ringStream)) return;

    // TODO magnificent diagnostics
    StringFormatter::send(stream, F("<X>\n"));
     DIAG(F("Command format <%<> failed CHECK(%S)"), matchedCommandFormat, checkFailedFormat);
     if (opcode >= ' ' && opcode <= '~') {
        DIAG(F("Opcode=%c params=%d"), opcode, params);
        for (int i = 0; i < params; i++)
            DIAG(F("p[%d]=%d (0x%x)"), i, p[i], p[i]);
      } else {
	        DIAG(F("Unprintable %x"), opcode);
      }
    
}

bool DCCEXParser::setThrottle(int16_t cab,int16_t tspeed,int16_t direction) {
    // Convert DCC-EX protocol speed steps where
     // -1=emergency stop, 0-126 as speeds
     // to DCC 0=stop, 1= emergency stop, 2-127 speeds
     if (tspeed > 126 || tspeed < -1) return false; // invalid JMRI speed code
     if (tspeed < 0) tspeed = 1; // emergency stop DCC speed
     else if (tspeed > 0) tspeed++; // map 1-126 -> 2-127
     if (cab == 0 && tspeed > 1) return false; // ignore broadcasts of speed>1
     if (direction < 0 || direction > 1) return false; // invalid direction code
     if (cab > 10239 || cab < 0) return false; // beyond DCC range

     DCC::setThrottle(cab, tspeed, direction);
     return true; 
}





//===================================

bool DCCEXParser::funcmap(int16_t cab, byte value, byte fstart, byte fstop)
{
  for (int16_t i = fstart; i <= fstop; i++) {
    if (! DCC::setFn(cab, i, value & 1)) return false;
    value >>= 1;
  }
  return true;
}

const FSH* DCCEXParser::matchedCommandFormat=nullptr;
const FSH* DCCEXParser::checkFailedFormat=nullptr; 


// Having broken the command into opcode and parameters, we now execute the command
// The actual commands and their parameter mappings are in DCCEXCommands.h
bool DCCEXParser::execute(byte * com,Print *stream, byte opcode,byte  params, int16_t p[], RingStream * ringStream) {
  #include "DCCEXCommands.h"
}

// CALLBACKS must be static
bool DCCEXParser::stashCallback(Print *stream, int16_t p[MAX_COMMAND_PARAMS], RingStream * ringStream)
{
    if (stashBusy )
        return false;
    stashBusy = true;
    stashStream = stream;
    stashRingStream=ringStream;
    if (ringStream) stashTarget= ringStream->peekTargetMark();
    memcpy(stashP, p, MAX_COMMAND_PARAMS * sizeof(p[0]));
    return true;
}

Print * DCCEXParser::getAsyncReplyStream() {
       if (stashRingStream) {
           stashRingStream->mark(stashTarget);
           return stashRingStream;
       }
       return stashStream;
}

void DCCEXParser::commitAsyncReplyStream() {
     if (stashRingStream) stashRingStream->commit();
     stashBusy = false;
}

void DCCEXParser::callback_W(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(),
          F("<r %d %d>\n"), stashP[0], result == 1 ? stashP[1] : -1);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_W4(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(),
	  F("<r%d|%d|%d %d>\n"), stashP[2], stashP[3], stashP[0], result == 1 ? stashP[1] : -1);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_B(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(), 
          F("<r%d|%d|%d %d %d>\n"), stashP[3], stashP[4], stashP[0], stashP[1], result == 1 ? stashP[2] : -1);
    commitAsyncReplyStream();
}
void DCCEXParser::callback_Vbit(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(), F("<v %d %d %d>\n"), stashP[0], stashP[1], result);
    commitAsyncReplyStream();
}
void DCCEXParser::callback_Vbyte(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(), F("<v %d %d>\n"), stashP[0], result);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_R(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(), F("<r%d|%d|%d %d>\n"), stashP[1], stashP[2], stashP[0], result);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_r(int16_t result)
{
    StringFormatter::send(getAsyncReplyStream(), F("<r %d %d %d >\n"), stashP[0], stashP[1], result);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_Rloco(int16_t result) {
  const FSH * detail;
  if (result<=0) {
    detail=F("<r %d>\n");
  } else {
    bool longAddr=result & LONG_ADDR_MARKER;        //long addr
    if (longAddr)
      result = result^LONG_ADDR_MARKER;
    if (longAddr && result <= HIGHEST_SHORT_ADDR)
      detail=F("<r LONG %d UNSUPPORTED>\n");
    else
      detail=F("<r %d>\n");
  }
  StringFormatter::send(getAsyncReplyStream(), detail, result);
  commitAsyncReplyStream();
}

void DCCEXParser::callback_Wloco(int16_t result)
{
    if (result==1) result=stashP[0]; // pick up original requested id from command
    StringFormatter::send(getAsyncReplyStream(), F("<w %d>\n"), result);
    commitAsyncReplyStream();
}

void DCCEXParser::callback_Wconsist(int16_t result)
{
    if (result==1) result=stashP[1]; // pick up original requested id from command
    StringFormatter::send(getAsyncReplyStream(), F("<w CONSIST %d%S>\n"),
     result, stashP[2]=="REVERSE"_hk ? F(" REVERSE") : F(""));
    commitAsyncReplyStream();
}
