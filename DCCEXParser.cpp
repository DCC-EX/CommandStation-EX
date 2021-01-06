/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  © 2020, Harald Barth.
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
#include "StringFormatter.h"
#include "DCCEXParser.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "Turnouts.h"
#include "Outputs.h"
#include "Sensors.h"
#include "freeMemory.h"
#include "GITHUB_SHA.h"
#include "version.h"

#include "EEStore.h"
#include "DIAG.h"

// These keywords are used in the <1> command. The number is what you get if you use the keyword as a parameter.
// To discover new keyword numbers , use the <$ YOURKEYWORD> command
const int HASH_KEYWORD_PROG = -29718;
const int HASH_KEYWORD_MAIN = 11339;
const int HASH_KEYWORD_JOIN = -30750;
const int HASH_KEYWORD_CABS = -11981;
const int HASH_KEYWORD_RAM = 25982;
const int HASH_KEYWORD_CMD = 9962;
const int HASH_KEYWORD_WIT = 31594;
const int HASH_KEYWORD_WIFI = -5583;
const int HASH_KEYWORD_ACK = 3113;
const int HASH_KEYWORD_ON = 2657;
const int HASH_KEYWORD_DCC = 6436;
const int HASH_KEYWORD_SLOW = -17209;
const int HASH_KEYWORD_PROGBOOST = -6353;
const int HASH_KEYWORD_EEPROM = -7168;
const int HASH_KEYWORD_LIMIT = 27413;
const int HASH_KEYWORD_ETHERNET = -30767;    
const int HASH_KEYWORD_MAX = 16244;
const int HASH_KEYWORD_MIN = 15978;

int DCCEXParser::stashP[MAX_PARAMS];
bool DCCEXParser::stashBusy;

Print *DCCEXParser::stashStream = NULL;

// This is a JMRI command parser, one instance per incoming stream
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
// Non-DCC things like turnouts, pins and sensors are handled in additional JMRI interface classes.

DCCEXParser::DCCEXParser() {}
void DCCEXParser::flush()
{
    if (Diag::CMD)
        DIAG(F("\nBuffer flush"));
    bufferLength = 0;
    inCommandPayload = false;
}

void DCCEXParser::loop(Stream &stream)
{
    while (stream.available())
    {
        if (bufferLength == MAX_BUFFER)
        {
            flush();
        }
        char ch = stream.read();
        if (ch == '<')
        {
            inCommandPayload = true;
            bufferLength = 0;
            buffer[0] = '\0';
        }
        else if (ch == '>')
        {
            buffer[bufferLength] = '\0';
            parse(&stream, buffer, false); // Parse this allowing async responses
            inCommandPayload = false;
            break;
        }
        else if (inCommandPayload)
        {
            buffer[bufferLength++] = ch;
        }
    }
    Sensor::checkAll(&stream); // Update and print changes
}

int DCCEXParser::splitValues(int result[MAX_PARAMS], const byte *cmd)
{
    byte state = 1;
    byte parameterCount = 0;
    int runningValue = 0;
    const byte *remainingCmd = cmd + 1; // skips the opcode
    bool signNegative = false;

    // clear all parameters in case not enough found
    for (int i = 0; i < MAX_PARAMS; i++)
        result[i] = 0;

    while (parameterCount < MAX_PARAMS)
    {
        byte hot = *remainingCmd;

        switch (state)
        {

        case 1: // skipping spaces before a param
            if (hot == ' ')
                break;
            if (hot == '\0' || hot == '>')
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
                runningValue = 10 * runningValue + (hot - '0');
                break;
            }
            if (hot >= 'A' && hot <= 'Z')
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

int DCCEXParser::splitHexValues(int result[MAX_PARAMS], const byte *cmd)
{
    byte state = 1;
    byte parameterCount = 0;
    int runningValue = 0;
    const byte *remainingCmd = cmd + 1; // skips the opcode
    
    // clear all parameters in case not enough found
    for (int i = 0; i < MAX_PARAMS; i++)
        result[i] = 0;

    while (parameterCount < MAX_PARAMS)
    {
        byte hot = *remainingCmd;

        switch (state)
        {

        case 1: // skipping spaces before a param
            if (hot == ' ')
                break;
            if (hot == '\0' || hot == '>')
                return parameterCount;
            state = 2;
            continue;

        case 2: // checking first hex digit
            runningValue = 0;
            state = 3;
            continue;

        case 3: // building a parameter
            if (hot >= '0' && hot <= '9')
            {
                runningValue = 16 * runningValue + (hot - '0');
                break;
            }
            if (hot >= 'A' && hot <= 'F')
            {
                runningValue = 16 * runningValue + 10 + (hot - 'A');
                break;
            }
            if (hot >= 'a' && hot <= 'f')
            {
                runningValue = 16 * runningValue + 10 + (hot - 'a');
                break;
            }
            if (hot==' ' || hot=='>' || hot=='\0') { 
               result[parameterCount] = runningValue;
               parameterCount++;
               state = 1;
               continue;
            }
            return -1; // invalid hex digit
        }
        remainingCmd++;
    }
    return parameterCount;
}

FILTER_CALLBACK DCCEXParser::filterCallback = 0;
FILTER_CALLBACK DCCEXParser::filterRMFTCallback = 0;
AT_COMMAND_CALLBACK DCCEXParser::atCommandCallback = 0;
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

// See documentation on DCC class for info on this section
void DCCEXParser::parse(Print *stream, byte *com, bool blocking)
{
    (void)EEPROM; // tell compiler not to warn this is unused
    if (Diag::CMD)
        DIAG(F("\nPARSING:%s\n"), com);
    int p[MAX_PARAMS];
    while (com[0] == '<' || com[0] == ' ')
        com++; // strip off any number of < or spaces
    byte params = splitValues(p, com);
    byte opcode = com[0];

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
        int cab;
        int tspeed;
        int direction;

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

        DCC::setThrottle(cab, tspeed, direction);
        if (params == 4)
            StringFormatter::send(stream, F("<T %d %d %d>"), p[0], p[2], p[3]);
        else
            StringFormatter::send(stream, F("<O>"));
        return;
    }
    case 'f': // FUNCTION <f CAB BYTE1 [BYTE2]>
        if (parsef(stream, params, p))
            return;
        break;

    case 'a': // ACCESSORY <a ADDRESS SUBADDRESS ACTIVATE>
        if (p[2] != (p[2] & 1))
            return;
        DCC::setAccessory(p[0], p[1], p[2] == 1);
        return;

    case 'T': // TURNOUT  <T ...>
        if (parseT(stream, params, p))
            return;
        break;

    case 'Z': // OUTPUT <Z ...>
        if (parseZ(stream, params, p))
            return;
        break;

    case 'S': // SENSOR <S ...>
        if (parseS(stream, params, p))
            return;
        break;

    case 'w': // WRITE CV on MAIN <w CAB CV VALUE>
        DCC::writeCVByteMain(p[0], p[1], p[2]);
        return;

    case 'b': // WRITE CV BIT ON MAIN <b CAB CV BIT VALUE>
        DCC::writeCVBitMain(p[0], p[1], p[2], p[3]);
        return;

    case 'M': // WRITE TRANSPARENT DCC PACKET MAIN <M REG X1 ... X9>
    case 'P': // WRITE TRANSPARENT DCC PACKET PROG <P REG X1 ... X9>
        // Re-parse the command using a hex-only splitter
        params=splitHexValues(p,com)-1; // drop REG
        if (params<1) break;  
        {
          byte packet[params];
          for (int i=0;i<params;i++) {
            packet[i]=(byte)p[i+1];
            if (Diag::CMD) DIAG(F("packet[%d]=%d (0x%x)\n"), i, packet[i], packet[i]);
          }
          (opcode=='M'?DCCWaveform::mainTrack:DCCWaveform::progTrack).schedulePacket(packet,params,3);  
        }
        return;
        
    case 'W': // WRITE CV ON PROG <W CV VALUE CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream, p))
            break;
        DCC::writeCVByte(p[0], p[1], callback_W, blocking);
        return;

    case 'V': // VERIFY CV ON PROG <V CV VALUE> <V CV BIT 0|1>
        if (params == 2)
        { // <V CV VALUE>
            if (!stashCallback(stream, p))
                break;
            DCC::verifyCVByte(p[0], p[1], callback_Vbyte, blocking);
            return;
        }
        if (params == 3)
        {
            if (!stashCallback(stream, p))
                break;
            DCC::verifyCVBit(p[0], p[1], p[2], callback_Vbit, blocking);
            return;
        }
        break;

    case 'B': // WRITE CV BIT ON PROG <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream, p))
            break;
        DCC::writeCVBit(p[0], p[1], p[2], callback_B, blocking);
        return;

    case 'R': // READ CV ON PROG
        if (params == 3)
        { // <R CV CALLBACKNUM CALLBACKSUB>
            if (!stashCallback(stream, p))
                break;
            DCC::readCV(p[0], callback_R, blocking);
            return;
        }
        if (params == 0)
        { // <R> New read loco id
            if (!stashCallback(stream, p))
                break;
            DCC::getLocoId(callback_Rloco, blocking);
            return;
        }
        break;

    case '1': // POWERON <1   [MAIN|PROG]>
    case '0': // POWEROFF <0 [MAIN | PROG] >
        if (params > 1)
            break;
        {
            POWERMODE mode = opcode == '1' ? POWERMODE::ON : POWERMODE::OFF;
            DCC::setProgTrackSyncMain(false); // Only <1 JOIN> will set this on, all others set it off
            if (params == 0)
            {
                DCCWaveform::mainTrack.setPowerMode(mode);
                DCCWaveform::progTrack.setPowerMode(mode);
		if (mode == POWERMODE::OFF)
		  DCC::setProgTrackBoost(false);  // Prog track boost mode will not outlive prog track off
                StringFormatter::send(stream, F("<p%c>"), opcode);
                return;
            }
            switch (p[0])
            {
            case HASH_KEYWORD_MAIN:
                DCCWaveform::mainTrack.setPowerMode(mode);
                StringFormatter::send(stream, F("<p%c MAIN>"), opcode);
                return;

            case HASH_KEYWORD_PROG:
                DCCWaveform::progTrack.setPowerMode(mode);
		if (mode == POWERMODE::OFF)
		  DCC::setProgTrackBoost(false);  // Prog track boost mode will not outlive prog track off
                StringFormatter::send(stream, F("<p%c PROG>"), opcode);
                return;
            case HASH_KEYWORD_JOIN:
                DCCWaveform::mainTrack.setPowerMode(mode);
                DCCWaveform::progTrack.setPowerMode(mode);
                if (mode == POWERMODE::ON)
                {
                    DCC::setProgTrackSyncMain(true);
                    StringFormatter::send(stream, F("<p1 JOIN>"), opcode);
                }
                else
                    StringFormatter::send(stream, F("<p0>"));
                return;
            }
            break;
        }
        return;

    case 'c': // READ CURRENT <c>
        StringFormatter::send(stream, F("<a %d>"), DCCWaveform::mainTrack.get1024Current());
        return;

    case 'Q': // SENSORS <Q>
        Sensor::printAll(stream);
        return;

    case 's': // <s>
        StringFormatter::send(stream, F("<p%d>"), DCCWaveform::mainTrack.getPowerMode() == POWERMODE::ON);
        StringFormatter::send(stream, F("<iDCC-EX V-%S / %S / %S G-%S>"), F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA));
        Turnout::printAll(stream); //send all Turnout states
        Output::printAll(stream);  //send all Output  states
        Sensor::printAll(stream);  //send all Sensor  states
        // TODO Send stats of  speed reminders table
        return;       

    case 'E': // STORE EPROM <E>
        EEStore::store();
        StringFormatter::send(stream, F("<e %d %d %d>"), EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        return;

    case 'e': // CLEAR EPROM <e>
        EEStore::clear();
        StringFormatter::send(stream, F("<O>"));
        return;

    case ' ': // < >
        StringFormatter::send(stream, F("\n"));
        return;

    case 'D': // < >
        if (parseD(stream, params, p))
            return;
        return;

    case '#': // NUMBER OF LOCOSLOTS <#>
        StringFormatter::send(stream, F("<# %d>"), MAX_LOCOS);
        return;

    case 'F': // New command to call the new Loco Function API <F cab func 1|0>
        if (Diag::CMD)
            DIAG(F("Setting loco %d F%d %S"), p[0], p[1], p[2] ? F("ON") : F("OFF"));
        DCC::setFn(p[0], p[1], p[2] == 1);
        return;

    case '+': // Complex Wifi interface command (not usual parse)
        if (atCommandCallback) {
          DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
          DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
          atCommandCallback(com);
          return;
        }
        break;

    default: //anything else will diagnose and drop out to <X>
        DIAG(F("\nOpcode=%c params=%d\n"), opcode, params);
        for (int i = 0; i < params; i++)
            DIAG(F("p[%d]=%d (0x%x)\n"), i, p[i], p[i]);
        break;

    } // end of opcode switch

    // Any fallout here sends an <X>
    StringFormatter::send(stream, F("<X>"));
}

bool DCCEXParser::parseZ(Print *stream, int params, int p[])
{

    switch (params)
    {
    
    case 2: // <Z ID ACTIVATE>
    {
        Output *o = Output::get(p[0]);
        if (o == NULL)
            return false;
        o->activate(p[1]);
        StringFormatter::send(stream, F("<Y %d %d>"), p[0], p[1]);
    }
        return true;

    case 3: // <Z ID PIN INVERT>
        if (!Output::create(p[0], p[1], p[2], 1))
          return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    case 1: // <Z ID>
        if (!Output::remove(p[0]))
          return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    case 0: // <Z> list Output definitions
    {
        bool gotone = false;
        for (Output *tt = Output::firstOutput; tt != NULL; tt = tt->nextOutput)
        {
            gotone = true;
            StringFormatter::send(stream, F("<Y %d %d %d %d>"), tt->data.id, tt->data.pin, tt->data.iFlag, tt->data.oStatus);
        }
        return gotone;
    }
    default:
        return false;
    }
}

//===================================
bool DCCEXParser::parsef(Print *stream, int params, int p[])
{
    // JMRI sends this info in DCC message format but it's not exactly
    //      convenient for other processing
    if (params == 2)
    {
        byte instructionField = p[1] & 0xE0;   // 1110 0000
        if (instructionField == 0x80)          // 1000 0000 Function group 1
        {
	    // Shuffle bits from order F0 F4 F3 F2 F1 to F4 F3 F2 F1 F0 
            byte normalized = (p[1] << 1 & 0x1e) | (p[1] >> 4 & 0x01);
            funcmap(p[0], normalized, 0, 4);
        }
        else if (instructionField == 0xA0)     // 1010 0000 Function group 2
        {
	    if (p[1] & 0x10)                   // 0001 0000 Bit selects F5toF8 / F9toF12
		funcmap(p[0], p[1], 5, 8);
	    else
		funcmap(p[0], p[1], 9, 12);
        }
    }
    if (params == 3)
    {
        if (p[1] == 222)
            funcmap(p[0], p[2], 13, 20);
        else if (p[1] == 223)
            funcmap(p[0], p[2], 21, 28);
    }
    (void)stream; // NO RESPONSE
    return true;
}

void DCCEXParser::funcmap(int cab, byte value, byte fstart, byte fstop)
{
    for (int i = fstart; i <= fstop; i++)
    {
        DCC::setFn(cab, i, value & 1);
        value >>= 1;
    }
}

//===================================
bool DCCEXParser::parseT(Print *stream, int params, int p[])
{
    switch (params)
    {
    case 0: // <T>  list turnout definitions
    {
        bool gotOne = false;
        for (Turnout *tt = Turnout::firstTurnout; tt != NULL; tt = tt->nextTurnout)
        {
            gotOne = true;
            StringFormatter::send(stream, F("<H %d %d %d %d>"), tt->data.id, tt->data.address, 
                tt->data.subAddress, (tt->data.tStatus & STATUS_ACTIVE)!=0);
        }
        return gotOne; // will <X> if none found
    }

    case 1: // <T id>  delete turnout
        if (!Turnout::remove(p[0]))
            return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    case 2: // <T id 0|1>  activate turnout
    {
        Turnout *tt = Turnout::get(p[0]);
        if (!tt)
            return false;
        tt->activate(p[1]);
        StringFormatter::send(stream, F("<H %d %d>"), tt->data.id, (tt->data.tStatus & STATUS_ACTIVE)!=0);
    }
        return true;

    case 3: // <T id addr subaddr>  define turnout
        if (!Turnout::create(p[0], p[1], p[2]))
            return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    default:
        return false; // will <x>
    }
}

bool DCCEXParser::parseS(Print *stream, int params, int p[])
{

    switch (params)
    {
    case 3: // <S id pin pullup>  create sensor. pullUp indicator (0=LOW/1=HIGH)
        if (!Sensor::create(p[0], p[1], p[2]))
          return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    case 1: // S id> remove sensor
        if (!Sensor::remove(p[0]))
          return false;
        StringFormatter::send(stream, F("<O>"));
        return true;

    case 0: // <S> list sensor definitions
	if (Sensor::firstSensor == NULL)
	    return false;
        for (Sensor *tt = Sensor::firstSensor; tt != NULL; tt = tt->nextSensor)
        {
            StringFormatter::send(stream, F("<Q %d %d %d>"), tt->data.snum, tt->data.pin, tt->data.pullUp);
        }
        return true;

    default: // invalid number of arguments
        break;
    }
    return false;
}

bool DCCEXParser::parseD(Print *stream, int params, int p[])
{
    if (params == 0)
        return false;
    bool onOff = (params > 0) && (p[1] == 1 || p[1] == HASH_KEYWORD_ON); // dont care if other stuff or missing... just means off
    switch (p[0])
    {
    case HASH_KEYWORD_CABS: // <D CABS>
        DCC::displayCabList(stream);
        return true;

    case HASH_KEYWORD_RAM: // <D RAM>
        StringFormatter::send(stream, F("\nFree memory=%d\n"), freeMemory());
        break;

    case HASH_KEYWORD_ACK: // <D ACK ON/OFF> <D ACK [LIMIT|MIN|MAX] Value>
	if (params >= 3) {
	    if (p[1] == HASH_KEYWORD_LIMIT) {
	      DCCWaveform::progTrack.setAckLimit(p[2]);
	      StringFormatter::send(stream, F("\nAck limit=%dmA\n"), p[2]);
	    } else if (p[1] == HASH_KEYWORD_MIN) {
	      DCCWaveform::progTrack.setMinAckPulseDuration(p[2]);
	      StringFormatter::send(stream, F("\nAck min=%dus\n"), p[2]);
	    } else if (p[1] == HASH_KEYWORD_MAX) {
	      DCCWaveform::progTrack.setMaxAckPulseDuration(p[2]);
	      StringFormatter::send(stream, F("\nAck max=%dus\n"), p[2]);
	    }
	} else {
	  StringFormatter::send(stream, F("\nAck diag %S\n"), onOff ? F("on") : F("off"));
	  Diag::ACK = onOff;
	}
        return true;

    case HASH_KEYWORD_CMD: // <D CMD ON/OFF>
        Diag::CMD = onOff;
        return true;

    case HASH_KEYWORD_WIFI: // <D WIFI ON/OFF>
        Diag::WIFI = onOff;
        return true;

   case HASH_KEYWORD_ETHERNET: // <D ETHERNET ON/OFF>
        Diag::ETHERNET = onOff;
        return true;

    case HASH_KEYWORD_WIT: // <D WIT ON/OFF>
        Diag::WITHROTTLE = onOff;
        return true;

    case HASH_KEYWORD_DCC:
        DCCWaveform::setDiagnosticSlowWave(params >= 1 && p[1] == HASH_KEYWORD_SLOW);
        return true;

    case HASH_KEYWORD_PROGBOOST:
        DCC::setProgTrackBoost(true);
	return true;

    case HASH_KEYWORD_EEPROM: // <D EEPROM NumEntries>
	if (params >= 2)
	    EEStore::dump(p[1]);
	return true;

    default: // invalid/unknown
        break;
    }
    return false;
}

// CALLBACKS must be static
bool DCCEXParser::stashCallback(Print *stream, int p[MAX_PARAMS])
{
    if (stashBusy )
        return false;
    stashBusy = true;
    stashStream = stream;
    memcpy(stashP, p, MAX_PARAMS * sizeof(p[0]));
    return true;
}
void DCCEXParser::callback_W(int result)
{
    StringFormatter::send(stashStream, F("<r%d|%d|%d %d>"), stashP[2], stashP[3], stashP[0], result == 1 ? stashP[1] : -1);
    stashBusy = false;
}

void DCCEXParser::callback_B(int result)
{
    StringFormatter::send(stashStream, F("<r%d|%d|%d %d %d>"), stashP[3], stashP[4], stashP[0], stashP[1], result == 1 ? stashP[2] : -1);
    stashBusy = false;
}
void DCCEXParser::callback_Vbit(int result)
{
    StringFormatter::send(stashStream, F("<v %d %d %d>"), stashP[0], stashP[1], result);
    stashBusy = false;
}
void DCCEXParser::callback_Vbyte(int result)
{
    StringFormatter::send(stashStream, F("<v %d %d>"), stashP[0], result);
    stashBusy = false;
}

void DCCEXParser::callback_R(int result)
{
    StringFormatter::send(stashStream, F("<r%d|%d|%d %d>"), stashP[1], stashP[2], stashP[0], result);
    stashBusy = false;
}

void DCCEXParser::callback_Rloco(int result)
{
    StringFormatter::send(stashStream, F("<r %d>"), result);
    stashBusy = false;
}
