/*
 *  © 2022 Paul M Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021 Herb Morton
 *  © 2020-2023 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2021 Fred Decker
 *  © 2020-2021 Chris Harlow
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
  k, Reserved for future use - Potentially Railcom
  K, Reserved for future use - Potentially Railcom
  l, Loco speedbyte/function map broadcast
  L, Reserved for LCC interface (implemented in EXRAIL)
  m,
  M, Write DCC packet
  n,
  N,
  o,
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
  X, Invalid command
  y,
  Y, Output broadcast
  z,
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

// This is a JMRI command parser.
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
// Non-DCC things like turnouts, pins and sensors are handled in additional JMRI interface classes.


int16_t DCCEXParser::splitValues(int16_t result[MAX_COMMAND_PARAMS], const byte *cmd, bool usehex)
{
    byte state = 1;
    byte parameterCount = 0;
    int16_t runningValue = 0;
    const byte *remainingCmd = cmd + 1; // skips the opcode
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

        case 2: // checking sign
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
                // Since JMRI got modified to send keywords in some rare cases, we need this
                // Super Kluge to turn keywords into a hash value that can be recognised later
                runningValue = ((runningValue << 5) + runningValue) ^ hot;
                break;
            }
            result[parameterCount] = runningValue * (signNegative ? -1 : 1);
            parameterCount++;
            state = 1;
            continue;
        }
        remainingCmd++;
    }
    return parameterCount;
}

extern __attribute__((weak))  void myFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[]);
FILTER_CALLBACK DCCEXParser::filterCallback = myFilter;
FILTER_CALLBACK DCCEXParser::filterRMFTCallback = 0;
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
    int16_t splitnum = splitValues(p, com, opcode=='M' || opcode=='P');
    if (splitnum < 0 || splitnum >= MAX_COMMAND_PARAMS) // if arguments are broken, leave but via printing <X>
      goto out;
    // Because of check above we are now inside byte size
    params = splitnum;

    if (filterCallback)
        filterCallback(stream, opcode, params, p);
    if (filterRMFTCallback && opcode!='\0')
        filterRMFTCallback(stream, opcode, params, p);

    // Functions return from this switch if complete, break from switch implies error <X> to send
    switch (opcode)
    {
    case '\0':
        return; // filterCallback asked us to ignore
    case 't':   // THROTTLE <t [REGISTER] CAB SPEED DIRECTION>
    {
        if (params==1) {  // <t cab>  display state
        
        int16_t slot=DCC::lookupSpeedTable(p[0],false);
        if (slot>=0) {
            DCC::LOCO * sp=&DCC::speedTable[slot];
            StringFormatter::send(stream,F("<l %d %d %d %l>\n"),
			sp->loco,slot,sp->speedCode,sp->functions);
            }
        else // send dummy state speed 0 fwd no functions. 
            StringFormatter::send(stream,F("<l %d -1 128 0>\n"),p[0]);
        return; 
        }
        
        int16_t cab;
        int16_t tspeed;
        int16_t direction;
        
        if (params == 4)
        { // <t REGISTER CAB SPEED DIRECTION>
            cab = p[1];
            tspeed = p[2];
            direction = p[3];
        }
        else if (params == 3)
        { // <t CAB SPEED DIRECTION>
            cab = p[0];
            tspeed = p[1];
            direction = p[2];
        }
        else
            break;

        // Convert DCC-EX protocol speed steps where
        // -1=emergency stop, 0-126 as speeds
        // to DCC 0=stop, 1= emergency stop, 2-127 speeds
        if (tspeed > 126 || tspeed < -1)
            break; // invalid JMRI speed code
        if (tspeed < 0)
            tspeed = 1; // emergency stop DCC speed
        else if (tspeed > 0)
            tspeed++; // map 1-126 -> 2-127
        if (cab == 0 && tspeed > 1)
            break; // ignore broadcasts of speed>1

        if (direction < 0 || direction > 1)
            break; // invalid direction code
	if (cab > 10239 || cab < 0)
	    break; // beyond DCC range

        DCC::setThrottle(cab, tspeed, direction);
        if (params == 4) // send obsolete format T response
            StringFormatter::send(stream, F("<T %d %d %d>\n"), p[0], p[2], p[3]);
        // speed change will be broadcast anyway in new <l > format
        return;
    }
    case 'f': // FUNCTION <f CAB BYTE1 [BYTE2]>
        if (parsef(stream, params, p))
            return;
        break;

    case 'a': // ACCESSORY <a ADDRESS SUBADDRESS ACTIVATE [ONOFF]> or <a LINEARADDRESS ACTIVATE>
        { 
          int address;
          byte subaddress;
          byte activep;
          byte onoff;
          if (params==2) { // <a LINEARADDRESS ACTIVATE>
              address=(p[0] - 1) / 4 + 1;
              subaddress=(p[0] - 1)  % 4;
              activep=1;
              onoff=2; // send both
          }
          else if (params==3) { // <a ADDRESS SUBADDRESS ACTIVATE>
              address=p[0];
              subaddress=p[1];
              activep=2;
              onoff=2; // send both
          }
          else if (params==4) { // <a ADDRESS SUBADDRESS ACTIVATE ONOFF>
              address=p[0];
              subaddress=p[1];
              activep=2;
	      if ((p[3] < 0) || (p[3] > 1))        // invalid onoff    0|1
		break;
              onoff=p[3];
          }
          else break; // invalid no of parameters
          
          if (
	      ((address & 0x01FF) != address)      // invalid address (limit 9 bits)
           || ((subaddress & 0x03) != subaddress)  // invalid subaddress (limit 2 bits)
           || (p[activep] > 1) || (p[activep] < 0) // invalid activate 0|1
	      ) break;
          // Honour the configuration option (config.h) which allows the <a> command to be reversed
#ifdef DCC_ACCESSORY_COMMAND_REVERSE
          DCC::setAccessory(address, subaddress,p[activep]==0,onoff);
#else
          DCC::setAccessory(address, subaddress,p[activep]==1,onoff);
#endif
        }
        return;
    
    case 'A': // EXTENDED ACCESSORY <A address value> 
        // Note: if this happens to match a defined EXRAIL 
        // DCCX_SIGNAL, then EXRAIL will have intercepted
        // this command alrerady.   
        if (params==2 && DCC::setExtendedAccessory(p[0],p[1])) return;
        break;
     
    case 'T': // TURNOUT  <T ...>
        if (parseT(stream, params, p))
            return;
        break;

    case 'z':  // direct pin manipulation
        if (p[0]==0) break; 
        if (params==1) {  // <z vpin | -vpin> 
            if (p[0]>0) IODevice::write(p[0],HIGH);
            else IODevice::write(-p[0],LOW);
            return;
        }
        if (params>=2 && params<=4) { // <z vpin ana;og profile duration> 
            // unused params default to 0           
            IODevice::writeAnalogue(p[0],p[1],p[2],p[3]);
            return;
        }
        break; 

    case 'Z': // OUTPUT <Z ...>
        if (parseZ(stream, params, p))
            return;
        break;

    case 'S': // SENSOR <S ...>
        if (parseS(stream, params, p))
            return;
        break;

#ifndef DISABLE_PROG
    case 'w': // WRITE CV on MAIN <w CAB CV VALUE>
      if (params != 3)
	break;
      DCC::writeCVByteMain(p[0], p[1], p[2]);
      return;

    case 'b': // WRITE CV BIT ON MAIN <b CAB CV BIT VALUE>
      if (params != 4)
	break;
      DCC::writeCVBitMain(p[0], p[1], p[2], p[3]);
      return;
#endif

    case 'M': // WRITE TRANSPARENT DCC PACKET MAIN <M REG X1 ... X9>
#ifndef DISABLE_PROG
    case 'P': // WRITE TRANSPARENT DCC PACKET PROG <P REG X1 ... X9>
#endif
        // NOTE: this command was parsed in HEX instead of decimal
        params--; // drop REG
        if (params<1) break;
	if (params > MAX_PACKET_SIZE) break;
        {
          byte packet[params];
          for (int i=0;i<params;i++) {
            packet[i]=(byte)p[i+1];
            if (Diag::CMD) DIAG(F("packet[%d]=%d (0x%x)"), i, packet[i], packet[i]);
          }
          (opcode=='M'?DCCWaveform::mainTrack:DCCWaveform::progTrack).schedulePacket(packet,params,3);  
        }
        return;
        
#ifndef DISABLE_PROG
    case 'W': // WRITE CV ON PROG <W CV VALUE CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream, p, ringStream))
	    break;
        if (params == 1) // <W id> Write new loco id (clearing consist and managing short/long)
            DCC::setLocoId(p[0],callback_Wloco);
        else if (params == 4)  // WRITE CV ON PROG <W CV VALUE [CALLBACKNUM] [CALLBACKSUB]>
            DCC::writeCVByte(p[0], p[1], callback_W4);
        else if (params == 2)  // WRITE CV ON PROG <W CV VALUE>
            DCC::writeCVByte(p[0], p[1], callback_W);
	else
            break;
        return;

    case 'V': // VERIFY CV ON PROG <V CV VALUE> <V CV BIT 0|1>
        if (params == 2)
        { // <V CV VALUE>
            if (!stashCallback(stream, p, ringStream))
                break;
            DCC::verifyCVByte(p[0], p[1], callback_Vbyte);
            return;
        }
        if (params == 3)
        {
            if (!stashCallback(stream, p, ringStream))
                break;
            DCC::verifyCVBit(p[0], p[1], p[2], callback_Vbit);
            return;
        }
        break;

    case 'B': // WRITE CV BIT ON PROG  <B CV BIT VALUE CALLBACKNUM CALLBACKSUB> or <B CV BIT VALUE>
        if (params != 3 && params != 5)
	  break;
        if (!stashCallback(stream, p, ringStream))
	  break;
        DCC::writeCVBit(p[0], p[1], p[2], callback_B);
        return;

    case 'R': // READ CV ON PROG
        if (params == 1)
        { // <R CV> -- uses verify callback
            if (!stashCallback(stream, p, ringStream))
                break;
            DCC::verifyCVByte(p[0], 0, callback_Vbyte);
            return;
        }
        if (params == 3)
        { // <R CV CALLBACKNUM CALLBACKSUB>
            if (!stashCallback(stream, p, ringStream))
                break;
            DCC::readCV(p[0], callback_R);
            return;
        }
        if (params == 0)
        { // <R> New read loco id
            if (!stashCallback(stream, p, ringStream))
                break;
            DCC::getLocoId(callback_Rloco);
            return;
        }
        break;
#endif

    case '1': // POWERON <1   [MAIN|PROG|JOIN]>
        {
	  if (params > 1) break;
	  if (params==0) { // All
	    TrackManager::setTrackPower(TRACK_MODE_ALL, POWERMODE::ON);
	  }
	  if (params==1) {
	    if (p[0]=="MAIN"_hk) { // <1 MAIN>
	      TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::ON);
            }
#ifndef DISABLE_PROG
            else if (p[0] == "JOIN"_hk) {  // <1 JOIN>
	      TrackManager::setJoin(true);
	      TrackManager::setTrackPower(TRACK_MODE_MAIN|TRACK_MODE_PROG, POWERMODE::ON);
            }
            else if (p[0]=="PROG"_hk) { // <1 PROG>
	      TrackManager::setJoin(false);
	      TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::ON);
            }
#endif
            else if (p[0] >= "A"_hk && p[0] <= "H"_hk) { // <1 A-H>
	      byte t = (p[0] - 'A');
	      TrackManager::setTrackPower(POWERMODE::ON, t);
	      //StringFormatter::send(stream, F("<p1 %c>\n"), t+'A');
            }
	    else break; // will reply <X>
	  }
	  //TrackManager::streamTrackState(NULL,t);
          
	  return;
	}
            
    case '0': // POWEROFF <0 [MAIN | PROG] >
        {
	  if (params > 1) break;
	  if (params==0) { // All
	    TrackManager::setJoin(false);
	    TrackManager::setTrackPower(TRACK_MODE_ALL, POWERMODE::OFF);
	  }
	  if (params==1) {
	    if (p[0]=="MAIN"_hk) { // <0 MAIN>
	      TrackManager::setJoin(false);
	      TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::OFF);
	    }
#ifndef DISABLE_PROG
            else if (p[0]=="PROG"_hk) { // <0 PROG>
	      TrackManager::progTrackBoosted=false;  // Prog track boost mode will not outlive prog track off
	      TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::OFF);
            }
#endif
	    else if (p[0] >= "A"_hk && p[0] <= "H"_hk) { // <1 A-H>
	      byte t = (p[0] - 'A');
	      TrackManager::setJoin(false);
	      TrackManager::setTrackPower(POWERMODE::OFF, t);
	      //StringFormatter::send(stream, F("<p0 %c>\n"), t+'A');
	    }
	    else break; // will reply <X>
	  }
	  return;
	}

    case '!': // ESTOP ALL  <!>
        DCC::setThrottle(0,1,1); // this broadcasts speed 1(estop) and sets all reminders to speed 1.
        return;

    case 'c': // SEND METER RESPONSES <c>
        // No longer useful because of multiple tracks See <JG> and <JI>
        if (params>0) break;
        TrackManager::reportObsoleteCurrent(stream);
        return;

    case 'Q': // SENSORS <Q>
        Sensor::printAll(stream);
        return;

    case 's': // STATUS <s>
        StringFormatter::send(stream, F("<iDCC-EX V-%S / %S / %S G-%S>\n"), F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA));
        CommandDistributor::broadcastPower(); // <s> is the only "get power status" command we have
        Turnout::printAll(stream); //send all Turnout states
        Sensor::printAll(stream);  //send all Sensor  states
        return;       

#ifndef DISABLE_EEPROM
    case 'E': // STORE EPROM <E>
        EEStore::store();
        StringFormatter::send(stream, F("<e %d %d %d>\n"), EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        return;

    case 'e': // CLEAR EPROM <e>
        EEStore::clear();
        StringFormatter::send(stream, F("<O>\n"));
        return;
#endif
    case ' ': // < >
        StringFormatter::send(stream, F("\n"));
        return;
    case 'C': // CONFIG <C [params]>
        if (parseC(stream, params, p))
            return;
        break;
#ifndef DISABLE_DIAG
    case 'D': // DIAG <D [params]>
        if (parseD(stream, params, p))
            return;
        break;
#endif
    case '=': // TRACK MANAGER CONTROL <= [params]>
        if (TrackManager::parseEqualSign(stream, params, p))
            return;
        break;

    case '#': // NUMBER OF LOCOSLOTS <#>
        StringFormatter::send(stream, F("<# %d>\n"), MAX_LOCOS);
        return;

    case '-': // Forget Loco <- [cab]>
        if (params > 1 || p[0]<0) break;
        if (p[0]==0) DCC::forgetAllLocos();
        else  DCC::forgetLoco(p[0]);
        return;

    case 'F': // New command to call the new Loco Function API <F cab func 1|0>
        if(params!=3) break; 
        if (Diag::CMD)
            DIAG(F("Setting loco %d F%d %S"), p[0], p[1], p[2] ? F("ON") : F("OFF"));
        if (DCC::setFn(p[0], p[1], p[2] == 1)) return;
	break;

#if WIFI_ON
    case '+': // Complex Wifi interface command (not usual parse)
        if (atCommandCallback && !ringStream) {
          TrackManager::setPower(POWERMODE::OFF);
          atCommandCallback((HardwareSerial *)stream,com);
          return;
        }
        break;
#endif 

    case 'J' : // throttle info access
        {
            if ((params<1) | (params>3)) break; // <J>
            //if ((params<1) | (params>2)) break; // <J>
            int16_t id=(params==2)?p[1]:0;
            switch(p[0]) {
                case "C"_hk: // <JC mmmm nn> sets time and speed
                    if (params==1) { // <JC> returns latest time
                        int16_t x = CommandDistributor::retClockTime();
                        StringFormatter::send(stream, F("<jC %d>\n"), x);
                        return;
                    }
                    CommandDistributor::setClockTime(p[1], p[2], 1);
                    return;
                
                case "G"_hk: // <JG> current gauge limits
                    if (params>1) break;
                    TrackManager::reportGauges(stream);   // <g limit...limit>     
                    return;
                
                case "I"_hk: // <JI> current values
                    if (params>1) break;
                    TrackManager::reportCurrent(stream);   // <g limit...limit>     
                    return;

                case "A"_hk: // <JA> intercepted by EXRAIL// <JA> returns automations/routes
                    if (params!=1) break; // <JA>
                    StringFormatter::send(stream, F("<jA>\n"));
                    return;
 
                case "M"_hk: // <JM> intercepted by EXRAIL
                    if (params>1) break; // invalid cant do
                    // <JM> requests stash size so say none.
                    StringFormatter::send(stream,F("<jM 0>\n")); 
                    return;
 
            case "R"_hk: // <JR> returns rosters 
                StringFormatter::send(stream, F("<jR"));
#ifdef EXRAIL_ACTIVE
                if (params==1) {
                    SENDFLASHLIST(stream,RMFT2::rosterIdList)
                }
                else {
                    auto rosterName= RMFT2::getRosterName(id);
                    if (!rosterName) rosterName=F("");

                    auto functionNames= RMFT2::getRosterFunctions(id);
                    if (!functionNames) functionNames=RMFT2::getRosterFunctions(0);
                    if (!functionNames) functionNames=F("");
                    StringFormatter::send(stream,F(" %d \"%S\" \"%S\""), 
					                            id, rosterName, functionNames);
                }
#endif          
                StringFormatter::send(stream, F(">\n"));      
                return; 
            case "T"_hk: // <JT> returns turnout list 
                StringFormatter::send(stream, F("<jT"));
                if (params==1) { // <JT>
                    for ( Turnout * t=Turnout::first(); t; t=t->next()) { 
                        if (t->isHidden()) continue;          
                        StringFormatter::send(stream, F(" %d"),t->getId());
                    }
                }
                else { // <JT id>
                    Turnout * t=Turnout::get(id);
                    if (!t || t->isHidden()) StringFormatter::send(stream, F(" %d X"),id);
                    else {
		      const FSH *tdesc = NULL;
#ifdef EXRAIL_ACTIVE
		      tdesc = RMFT2::getTurnoutDescription(id);
#endif
		      if (tdesc == NULL)
			tdesc = F("");
		      StringFormatter::send(stream, F(" %d %c \"%S\""),
					    id,t->isThrown()?'T':'C',
					    tdesc);
		    }
                }
                StringFormatter::send(stream, F(">\n"));
                return;
// No turntables without HAL support
#ifndef IO_NO_HAL
            case "O"_hk: // <JO returns turntable list
                StringFormatter::send(stream, F("<jO"));
                if (params==1) { // <JO>
                    for (Turntable * tto=Turntable::first(); tto; tto=tto->next()) { 
                        if (tto->isHidden()) continue;          
                        StringFormatter::send(stream, F(" %d"),tto->getId());
                    }
                    StringFormatter::send(stream, F(">\n"));
                } else {    // <JO id>
                    Turntable *tto=Turntable::get(id);
                    if (!tto || tto->isHidden()) {
                        StringFormatter::send(stream, F(" %d X>\n"), id);
                    } else {
                        uint8_t pos = tto->getPosition();
                        uint8_t type = tto->isEXTT();
                        uint8_t posCount = tto->getPositionCount();
                        const FSH *todesc = NULL;
#ifdef EXRAIL_ACTIVE
                        todesc = RMFT2::getTurntableDescription(id);
#endif
                        if (todesc == NULL) todesc = F("");
                        StringFormatter::send(stream, F(" %d %d %d %d \"%S\">\n"), id, type, pos, posCount, todesc);
                    }
                }
                return;
            case "P"_hk: // <JP id> returns turntable position list for the turntable id
                if (params==2) { // <JP id>
                    Turntable *tto=Turntable::get(id);
                    if (!tto || tto->isHidden()) {
                        StringFormatter::send(stream, F(" %d X>\n"), id);
                    } else {
                        uint8_t posCount = tto->getPositionCount();
                        const FSH *tpdesc = NULL;
                        for (uint8_t p = 0; p < posCount; p++) {
                            StringFormatter::send(stream, F("<jP"));
                            int16_t angle = tto->getPositionAngle(p);
#ifdef EXRAIL_ACTIVE
                            tpdesc = RMFT2::getTurntablePositionDescription(id, p);
#endif
                            if (tpdesc == NULL) tpdesc = F("");
                            StringFormatter::send(stream, F(" %d %d %d \"%S\""), id, p, angle, tpdesc);
                            StringFormatter::send(stream, F(">\n"));
                        }
                    }
                } else {
                    StringFormatter::send(stream, F("<jP X>\n"));
                }
                return;
#endif
            default: break;    
            }  // switch(p[1])
        break; // case J
        }

// No turntables without HAL support
#ifndef IO_NO_HAL
    case 'I': // TURNTABLE  <I ...>
        if (parseI(stream, params, p))
            return;
        break;
#endif

    case 'L': // LCC interface implemented in EXRAIL parser
        break; // Will <X> if not intercepted by EXRAIL 

#ifndef DISABLE_VDPY
    case '@': // JMRI saying "give me virtual LCD msgs"
        CommandDistributor::setVirtualLCDSerial(stream);
        StringFormatter::send(stream,
            F("<@ 0 0 \"DCC-EX v" VERSION "\">\n"
               "<@ 0 1 \"Lic GPLv3\">\n"));
        return; 
#endif
    default: //anything else will diagnose and drop out to <X>
      if (opcode >= ' ' && opcode <= '~') {
        DIAG(F("Opcode=%c params=%d"), opcode, params);
        for (int i = 0; i < params; i++)
            DIAG(F("p[%d]=%d (0x%x)"), i, p[i], p[i]);
      } else {
	DIAG(F("Unprintable %x"), opcode);
      }
      break;

    } // end of opcode switch

out:// Any fallout here sends an <X>
    StringFormatter::send(stream, F("<X>\n"));
}

bool DCCEXParser::parseZ(Print *stream, int16_t params, int16_t p[])
{

    switch (params)
    {
    
    case 2: // <Z ID ACTIVATE>
    {
        Output *o = Output::get(p[0]);
        if (o == NULL)
            return false;
        o->activate(p[1]);
        StringFormatter::send(stream, F("<Y %d %d>\n"), p[0], p[1]);
    }
        return true;

    case 3: // <Z ID PIN IFLAG>
        if (p[0] < 0 || p[2] < 0 || p[2] > 7 )
	        return false;
        if (!Output::create(p[0], p[1], p[2], 1))
          return false;
        StringFormatter::send(stream, F("<O>\n"));
        return true;

    case 1: // <Z ID>
        if (!Output::remove(p[0]))
          return false;
        StringFormatter::send(stream, F("<O>\n"));
        return true;

    case 0: // <Z> list Output definitions
    {
        bool gotone = false;
        for (Output *tt = Output::firstOutput; tt != NULL; tt = tt->nextOutput)
        {
            gotone = true;
            StringFormatter::send(stream, F("<Y %d %d %d %d>\n"), tt->data.id, tt->data.pin, tt->data.flags, tt->data.active);
        }
        return gotone;
    }
    default:
        return false;
    }
}

//===================================
bool DCCEXParser::parsef(Print *stream, int16_t params, int16_t p[])
{
  // JMRI sends this info in DCC message format but it's not exactly
  // convenient for other processing
  if (params == 2) {
    byte instructionField = p[1] & 0xE0;   // 1110 0000
    if (instructionField == 0x80) {        // 1000 0000 Function group 1
      // Shuffle bits from order F0 F4 F3 F2 F1 to F4 F3 F2 F1 F0
      byte normalized = (p[1] << 1 & 0x1e) | (p[1] >> 4 & 0x01);
      return (funcmap(p[0], normalized, 0, 4));
    } else if (instructionField == 0xA0) { // 1010 0000 Function group 2
      if (p[1] & 0x10)                     // 0001 0000 Bit selects F5toF8 / F9toF12
	return (funcmap(p[0], p[1], 5, 8));
      else
	return (funcmap(p[0], p[1], 9, 12));
    } 
  }
  if (params == 3) {
    if (p[1] == 222) {
      return (funcmap(p[0], p[2], 13, 20));
    } else if (p[1] == 223) {
      return (funcmap(p[0], p[2], 21, 28));
    } 
  }
  (void)stream; // NO RESPONSE
  return false;
}

bool DCCEXParser::funcmap(int16_t cab, byte value, byte fstart, byte fstop)
{
  for (int16_t i = fstart; i <= fstop; i++) {
    if (! DCC::setFn(cab, i, value & 1)) return false;
    value >>= 1;
  }
  return true;
}

//===================================
bool DCCEXParser::parseT(Print *stream, int16_t params, int16_t p[])
{
    switch (params)
    {
    case 0: // <T>  list turnout definitions
        return Turnout::printAll(stream); // will <X> if none found

    case 1: // <T id>  delete turnout
        if (!Turnout::remove(p[0]))
            return false;
        StringFormatter::send(stream, F("<O>\n"));
        return true;

    case 2: // <T id 0|1|T|C> 
        {
          bool state = false;
          switch (p[1]) {
            // Turnout messages use 1=throw, 0=close.
            case 0:
            case "C"_hk:
              state = true;
              break;
            case 1:
            case "T"_hk:
              state= false;
              break;
            case "X"_hk:
	    {
              Turnout *tt = Turnout::get(p[0]);
              if (tt) {
                tt->print(stream);
                return true;
              }
              return false;
	    }
            default: // Invalid parameter
	      return false;
          }
          if (!Turnout::setClosed(p[0], state)) return false;
          return true;
        }

    default: // Anything else is some kind of turnout create function.
      if (params == 6 && p[1] == "SERVO"_hk) { // <T id SERVO n n n n>
        if (!ServoTurnout::create(p[0], (VPIN)p[2], (uint16_t)p[3], (uint16_t)p[4], (uint8_t)p[5]))
          return false;
      } else 
      if (params == 3 && p[1] == "VPIN"_hk) { // <T id VPIN n>
        if (!VpinTurnout::create(p[0], p[2])) return false;
      } else 
      if (params >= 3 && p[1] == "DCC"_hk) {
        // <T id DCC addr subadd>   0<=addr<=511, 0<=subadd<=3 (like <a> command).<T>
        if (params==4 && p[2]>=0 && p[2]<512 && p[3]>=0 && p[3]<4) { // <T id DCC n m>
          if (!DCCTurnout::create(p[0], p[2], p[3])) return false;
        } else if (params==3 && p[2]>0 && p[2]<=512*4) { // <T id DCC nn>, 1<=nn<=2048
          // Linearaddress 1 maps onto decoder address 1/0 (not 0/0!).
          if (!DCCTurnout::create(p[0], (p[2]-1)/4+1, (p[2]-1)%4)) return false;
        } else
          return false;
      } else 
      if (params==3) { // legacy <T id addr subadd> for DCC accessory
        if (p[1]>=0 && p[1]<512 && p[2]>=0 && p[2]<4) {
          if (!DCCTurnout::create(p[0], p[1], p[2])) return false;
        } else
          return false;
      } 
      else 
      if (params==4) { // legacy <T id n n n> for Servo
        if (!ServoTurnout::create(p[0], (VPIN)p[1], (uint16_t)p[2], (uint16_t)p[3], 1)) return false;
      } else
        return false;

      StringFormatter::send(stream, F("<O>\n"));
      return true;
    }
}

bool DCCEXParser::parseS(Print *stream, int16_t params, int16_t p[])
{

    switch (params)
    {
    case 3: // <S id pin pullup>  create sensor. pullUp indicator (0=LOW/1=HIGH)
        if (!Sensor::create(p[0], p[1], p[2]))
          return false;
        StringFormatter::send(stream, F("<O>\n"));
        return true;

    case 1: // S id> remove sensor
        if (!Sensor::remove(p[0]))
          return false;
        StringFormatter::send(stream, F("<O>\n"));
        return true;

    case 0: // <S> list sensor definitions
      if (Sensor::firstSensor == NULL)
        return false;
      for (Sensor *tt = Sensor::firstSensor; tt != NULL; tt = tt->nextSensor)
      {
          StringFormatter::send(stream, F("<Q %d %d %d>\n"), tt->data.snum, tt->data.pin, tt->data.pullUp);
      }
      return true;

    default: // invalid number of arguments
        break;
    }
    return false;
}

bool DCCEXParser::parseC(Print *stream, int16_t params, int16_t p[]) {
    (void)stream; // arg not used, maybe later?
    if (params == 0)
        return false;
    switch (p[0])
    {
#ifndef DISABLE_PROG
    case "PROGBOOST"_hk:
        TrackManager::progTrackBoosted=true;
	    return true;
#endif
    case "RESET"_hk:
        DCCTimer::reset();
        break; // and <X> if we didnt restart
    case "SPEED28"_hk:
        DCC::setGlobalSpeedsteps(28);
	DIAG(F("28 Speedsteps"));
        return true;

    case "SPEED128"_hk:
        DCC::setGlobalSpeedsteps(128);
	DIAG(F("128 Speedsteps"));
        return true;
#if defined(HAS_ENOUGH_MEMORY) && !defined(ARDUINO_ARCH_UNO)
    case "RAILCOM"_hk:
        {   // <C RAILCOM ON|OFF|DEBUG >
            if (params<2) return false;
            bool on=false;
            bool debug=false;
            switch (p[1]) {
                case "ON"_hk:
                case 1:
                    on=true;
                    break;
                case "DEBUG"_hk:
                    on=true;
                    debug=true;
                    break;
                case "OFF"_hk:
                case 0:
                     break;
                default:
                 return false;
            }              
        DIAG(F("Railcom %S")
            ,DCCWaveform::setRailcom(on,debug)?F("ON"):F("OFF"));
        return true;     
        }
#endif
#ifndef DISABLE_PROG
    case "ACK"_hk: // <D ACK ON/OFF> <D ACK [LIMIT|MIN|MAX|RETRY] Value>
	if (params >= 3) {
	    if (p[1] == "LIMIT"_hk) {
	      DCCACK::setAckLimit(p[2]);
	      LCD(1, F("Ack Limit=%dmA"), p[2]);  // <D ACK LIMIT 42>
	    } else if (p[1] == "MIN"_hk) {
	      DCCACK::setMinAckPulseDuration(p[2]);
	      LCD(0, F("Ack Min=%uus"), p[2]);  //   <D ACK MIN 1500>
	    } else if (p[1] == "MAX"_hk) {
	      DCCACK::setMaxAckPulseDuration(p[2]);
	      LCD(0, F("Ack Max=%uus"), p[2]);  //   <D ACK MAX 9000>
	    } else if (p[1] == "RETRY"_hk) {
	      if (p[2] >255) p[2]=3;
	      LCD(0, F("Ack Retry=%d Sum=%d"), p[2], DCCACK::setAckRetry(p[2]));  //   <D ACK RETRY 2>
	    }
	} else {
      bool onOff = (params > 0) && (p[1] == 1 || p[1] == "ON"_hk); // dont care if other stuff or missing... just means off
    
	  DIAG(F("Ack diag %S"), onOff ? F("on") : F("off"));
	  Diag::ACK = onOff;
	}
        return true;
#endif

default: // invalid/unknown
      break;
    }
    return false;
}

bool DCCEXParser::parseD(Print *stream, int16_t params, int16_t p[])
{
    if (params == 0)
        return false;
    bool onOff = (params > 0) && (p[1] == 1 || p[1] == "ON"_hk); // dont care if other stuff or missing... just means off
    switch (p[0])
    {
    case "CABS"_hk: // <D CABS>
        DCC::displayCabList(stream);
        return true;

    case "RAM"_hk: // <D RAM>
        DIAG(F("Free memory=%d"), DCCTimer::getMinimumFreeMemory());
        return true;

    case "CMD"_hk: // <D CMD ON/OFF>
        Diag::CMD = onOff;
        return true;

#ifdef HAS_ENOUGH_MEMORY
    case "WIFI"_hk: // <D WIFI ON/OFF>
        Diag::WIFI = onOff;
        return true;

    case "ETHERNET"_hk: // <D ETHERNET ON/OFF>
        Diag::ETHERNET = onOff;
        return true;

    case "WIT"_hk: // <D WIT ON/OFF>
        Diag::WITHROTTLE = onOff;
        return true;

    case "LCN"_hk: // <D LCN ON/OFF>
        Diag::LCN = onOff;
        return true;
#endif
#ifndef DISABLE_EEPROM
    case "EEPROM"_hk: // <D EEPROM NumEntries>
	if (params >= 2)
	    EEStore::dump(p[1]);
	return true;
#endif
    case "SERVO"_hk:  // <D SERVO vpin position [profile]>

    case "ANOUT"_hk:  // <D ANOUT vpin position [profile]>
        IODevice::writeAnalogue(p[1], p[2], params>3 ? p[3] : 0);
        return true;

    case "ANIN"_hk:   // <D ANIN vpin>  Display analogue input value
        DIAG(F("VPIN=%u value=%d"), p[1], IODevice::readAnalogue(p[1]));
        return true;

#if !defined(IO_NO_HAL)
    case "HAL"_hk: 
        if (p[1] == "SHOW"_hk) 
          IODevice::DumpAll();
        else if (p[1] == "RESET"_hk)
          IODevice::reset();
        return true;
#endif

    case "TT"_hk:     // <D TT vpin steps activity>
        IODevice::writeAnalogue(p[1], p[2], params>3 ? p[3] : 0);
        return true;

    default: // invalid/unknown
        return parseC(stream, params, p);
    }
    return false;
}

// ==========================
// Turntable - no support if no HAL
// <I> - list all
// <I id> - broadcast type and current position
// <I id DCC> - create DCC - This is TBA
// <I id steps> - operate (DCC)
// <I id steps activity> - operate (EXTT)
// <I id ADD position value> - add position
// <I id EXTT i2caddress vpin home> - create EXTT
#ifndef IO_NO_HAL
bool DCCEXParser::parseI(Print *stream, int16_t params, int16_t p[])
{
    switch (params)
    {
    case 0: // <I> list turntable objects
        return Turntable::printAll(stream);

    case 1: // <I id> broadcast type and current position
        {    
            Turntable *tto = Turntable::get(p[0]);
            if (tto) {
                bool type = tto->isEXTT();
                uint8_t position = tto->getPosition();
                StringFormatter::send(stream, F("<I %d %d>\n"), type, position);
            } else {
                return false;
            }
        }
        return true;
    
    case 2: // <I id position> - rotate a DCC turntable
        {
            Turntable *tto = Turntable::get(p[0]);
            if (tto && !tto->isEXTT()) {
                if (!tto->setPosition(p[0], p[1])) return false;
            } else {
                return false;
            }
        }
        return true;

    case 3: // <I id position activity> | <I id DCC home> - rotate to position for EX-Turntable or create DCC turntable
        {
            Turntable *tto = Turntable::get(p[0]);
            if (p[1] == "DCC"_hk) {
                if (tto || p[2] < 0 || p[2] > 3600) return false;
                if (!DCCTurntable::create(p[0])) return false;
                Turntable *tto = Turntable::get(p[0]);
                tto->addPosition(0, 0, p[2]);
                StringFormatter::send(stream, F("<I>\n"));
            } else {
                if (!tto) return false;
                if (!tto->isEXTT()) return false;
                if (!tto->setPosition(p[0], p[1], p[2])) return false;
            }
        }
        return true;
    
    case 4: // <I id EXTT vpin home> create an EXTT turntable
        {
            Turntable *tto = Turntable::get(p[0]);
            if (p[1] == "EXTT"_hk) {
                if (tto || p[3] < 0 || p[3] > 3600) return false;
                if (!EXTTTurntable::create(p[0], (VPIN)p[2])) return false;
                Turntable *tto = Turntable::get(p[0]);
                tto->addPosition(0, 0, p[3]);
                StringFormatter::send(stream, F("<I>\n"));
            } else {
                return false;
            }
        }
        return true;
    
    case 5: // <I id ADD position value angle> add a position
        {
            Turntable *tto = Turntable::get(p[0]);
            if (p[1] == "ADD"_hk) {
                // tto must exist, no more than 48 positions, angle 0 - 3600
                if (!tto || p[2] > 48 || p[4] < 0 || p[4] > 3600) return false;
                tto->addPosition(p[2], p[3], p[4]);
                StringFormatter::send(stream, F("<I>\n"));
            } else {
                return false;
            }
        }
        return true;
    
    default:    // Anything else is invalid
        return false;
    }
}
#endif

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
