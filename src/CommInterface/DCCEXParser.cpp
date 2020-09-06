/*
 *  DCCEXParser.cpp
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DCCEXParser.h"

#include <inttypes.h>

#include "../Accessories/EEStore.h"
#include "../Accessories/Outputs.h"
#include "../Accessories/Sensors.h"
#include "../Accessories/Turnouts.h"
#include "../../Config.h"
#include "CommManager.h"
#include "../WiFiInterface/WiFiInterface.h"

DCC* DCCEXParser::mainTrack;
DCC* DCCEXParser::progTrack;

int DCCEXParser::p[MAX_PARAMS];

// These keywords are used in the <1> command. The number is what you get if you 
// use the keyword as a parameter. To discover new keyword numbers , use the 
// <$ YOURKEYWORD> command
const int HASH_KEYWORD_PROG = -29718;
const int HASH_KEYWORD_MAIN = 11339;
const int HASH_KEYWORD_JOIN = -30750;

void DCCEXParser::init(DCC* mainTrack_, DCC* progTrack_) {
  mainTrack = mainTrack_;
  progTrack = progTrack_;
} 

int DCCEXParser::stringParser(const char *com, int result[]) {
  byte state=1;
  byte parameterCount=0;
  int runningValue=0;
  const char * remainingCmd=com;  // skips the opcode
  bool signNegative=false;
  
  // clear all parameters in case not enough found
  for (int i=0;i<MAX_PARAMS;i++) result[i]=0;
  
  while(parameterCount<MAX_PARAMS) {
    char hot=*remainingCmd;
    switch (state) {
    case 1: // skipping spaces before a param
      if (hot==' ') break;
      if (hot == '\0' || hot=='>') return parameterCount;
      state=2;
      continue;
    case 2: // checking sign
      signNegative=false;
      runningValue=0;
      state=3; 
      if (hot!='-') continue; 
      signNegative=true;
      break; 
    case 3: // building a parameter   
      if (hot>='0' && hot<='9') {
        runningValue=10*runningValue+(hot-'0');
        break;
      }
      if (hot>='A' && hot<='Z') {
        // Since JMRI got modified to send keywords in some rare cases, we need this
        // Super Kluge to turn keywords into a hash value that can be recognised later
        runningValue = ((runningValue << 5) + runningValue) ^ hot;
        break;
      }
      result[parameterCount] = runningValue * (signNegative ?-1:1);
      parameterCount++;
      state=1; 
      continue;
    }   
    remainingCmd++;
  }
  return parameterCount;
}

// See documentation on DCC class for info on this section
void DCCEXParser::parse(Print* stream, const char *com) {
  if (com[0]=='<') com++;

  int numArgs = stringParser(com+1, p);
  
  switch(com[0]) {
  
/***** SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL ****/

  case 't': {      // <t REGISTER CAB SPEED DIRECTION>
    genericResponse throttleResponse;
    if (numArgs!=4) break;  
    
    int speed=p[2];
    if (speed>126 || speed<-1) break; // invalid JMRI speed code
    if (speed<0) speed=1; // emergency stop DCC speed
    else if (speed>0) speed++; // map 1-126 -> 2-127
    
    if (p[1]==0 && speed>1) break; // ban broadcast movement
    if (p[3]<0 || p[3]>1) break;      // invalid direction code
    
    uint8_t speedCode = (speed & 0x7F) + (p[3]==0 ? 0 : 128);

    if(mainTrack->setThrottle(p[1], speedCode, throttleResponse) == ERR_OK)
    CommManager::send(stream, F("<T %d %d %d>"), p[0], p[2], p[3]);
    
    break;
  }
  
/***** OPERATE ENGINE DECODER FUNCTIONS F0-F28 ****/

  case 'f': {       // <f CAB BYTE1 [BYTE2]>    
    if (numArgs==2) {
      uint8_t groupcode=p[1] & 0xE0;
      if (groupcode == 0x80) {
        uint8_t normalized = (p[1]<<1 & 0x1e ) | (p[1]>>4 & 0x01);
        functionMap(p[0],normalized,0,4);
      }
      else if (groupcode == 0xC0) {
        functionMap(p[0],p[1],5,8);
      }
      else if (groupcode == 0xA0) {
        functionMap(p[0],p[1],9,12);
      }
    }   
    if (numArgs==3) { 
      if (p[1]==222) functionMap(p[0],p[2],13,20);
      else if (p[1]==223) functionMap(p[0],p[2],21,28);
    }
    
    // TODO use response?
    
    break;
  }

/***** OPERATE STATIONARY ACCESSORY DECODERS  ****/

  case 'a': {      // <a ADDRESS SUBADDRESS ACTIVATE>        
    genericResponse response;

    mainTrack->setAccessory(p[0], p[1], p[2], response);
    
    break;
  }
  
/***** CREATE/EDIT/REMOVE/SHOW & OPERATE A TURN-OUT  ****/

  case 'T':       // <T ID THROW>      
    Turnout *t;

    switch(numArgs){

    // argument is string with id number of turnout followed by zero (not 
    // thrown) or one (thrown), set state of turnout
    case 2:   
      t=Turnout::get(p[0]);
      if(t!=NULL) {
        t->activate(p[1], (DCC*) mainTrack);
        CommManager::send(stream, F("<H %d %d>"), t->data.id, t->data.tStatus);
      } else
        CommManager::send(stream, F("<X>"));
      break;

    // argument is string with id number of turnout followed by an address and 
    //   subAddress, then create the turnout
    case 3:                     
      Turnout::create(p[0],p[1],p[2],1);
      break;

    case 1:        // argument is a string with id number only, remove turnout
      if (Turnout::remove(p[0])) {
        CommManager::send(stream, F("<O>"));
      } else {
        CommManager::send(stream, F("<X>"));
      }
      break;

    case 0:                    // no arguments, list turnouts
      if(Turnout::firstTurnout==NULL){
        CommManager::send(stream, F("<X>"));
        break;
      }
      for(Turnout *tt=Turnout::firstTurnout;tt!=NULL;tt=tt->nextTurnout){
        CommManager::send(stream, F("<H %d %d %d %d>"), tt->data.id, tt->data.address, 
          tt->data.subAddress, tt->data.tStatus);
      }
      break;

    }
    
    break;
  
/***** CREATE/EDIT/REMOVE/SHOW & OPERATE AN OUTPUT PIN  ****/

  case 'Z':       // <Z ID ACTIVATE>
    Output* o;

    switch(numArgs){
    
    // argument is string with id number of output followed by zero (LOW) or 
    // one (HIGH)
    case 2:                     
      o=Output::get(p[0]);
      if(o!=NULL)
        o->activate(stream, p[1]);
      else
        CommManager::send(stream, F("<X>"));
      break;

    // argument is string with id number of output followed by a pin number and 
    // invert flag
    case 3:                     
      Output::create(stream, p[0],p[1],p[2],1);
      break;

    case 1:                     // argument is a string with id number only
      Output::remove(stream, p[0]);
      break;

    case 0:                    // no arguments
      Output::show(stream, 1);                  // verbose show
      break;
    }
    
    break;
  
/***** CREATE/EDIT/REMOVE/SHOW A SENSOR  ****/

  case 'S':
    switch(numArgs){

    // argument is string with id number of sensor followed by a pin number and 
    // pullUp indicator (0=LOW/1=HIGH)
    case 3:                     
      Sensor::create(stream, p[0],p[1],p[2],1);
      break;

    case 1:                     // argument is a string with id number only
      Sensor::remove(stream, p[0]);
      break;

    case 0:                    // no arguments
      Sensor::show(stream);
      break;

    case 2:                     // invalid number of arguments
      CommManager::send(stream, F("<X>"));
      break;
    }
  
    break;

/***** SHOW STATUS OF ALL SENSORS ****/

  case 'Q':         // <Q>
    Sensor::status(stream);
    break;


/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON MAIN TRACK  ****/

  case 'w': {     // <w CAB CV VALUE>
    genericResponse response;

    mainTrack->writeCVByteMain(p[0], p[1], p[2], response, stream, POMResponse);
    
    break;
  }

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON MAIN TRACK  ****/

  case 'b': {     // <b CAB CV BIT VALUE>
    genericResponse response;

    mainTrack->writeCVBitMain(p[0], p[1], p[2], p[3], response, stream, POMResponse);
    
    break;
  }

/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON PROG TRACK  ****/

  case 'W':      // <W CV VALUE CALLBACKNUM CALLBACKSUB>

    progTrack->writeCVByte(p[0], p[1], p[2], p[3], stream, cvResponse);

    break;

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON PROG TRACK  ****/

  case 'B':      // <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
    
    progTrack->writeCVBit(p[0], p[1], p[2], p[3], p[4], stream, cvResponse);
    
    break;

/***** READ CONFIGURATION VARIABLE BYTE FROM ENGINE DECODER ON PROG TRACK  ****/

  case 'R':     // <R CV CALLBACKNUM CALLBACKSUB>        
    progTrack->readCV(p[0], p[1], p[2], stream, cvResponse);

    break;

/***** READ CONFIGURATION VARIABLE BYTE FROM RAILCOM DECODER ON MAIN TRACK ****/
  
  case 'r': {   // <r CAB CV>
    genericResponse response;

    mainTrack->readCVByteMain(p[0], p[1], response, stream, POMResponse);
    break;
    }

/***** READ 4 CONFIGURATION VARIABLE BYTES FROM RAILCOM DECODER ON MAIN  ****/

  case 'm': { // <m CAB CV>
    genericResponse response;

    mainTrack->readCVBytesMain(p[0], p[1], response, stream, POMResponse);
    break;
    }

/***** TURN ON/OFF POWER FROM MOTOR SHIELD TO TRACKS  ****/

  case '0':       // <0 [MAIN/PROG/JOIN]>
  case '1':       // <1 [MAIN/PROG/JOIN]>
    {
      if (numArgs > 1) break;
      bool on = com[0] == '1' ? true : false;
      if (numArgs==0) {
        mainTrack->board->power(on, false);
        progTrack->board->power(on, false);
        CommManager::broadcast(F("<p%c>"), com[0]);
        break;
      }
      switch (p[0]) {
      case HASH_KEYWORD_MAIN:
        mainTrack->board->power(on, false);
        CommManager::broadcast(F("<p%c MAIN>"), com[0]);
        return;
      case HASH_KEYWORD_PROG:
        mainTrack->board->power(on, false);
        CommManager::broadcast(F("<p%c PROG>"), com[0]);
        return;
      case HASH_KEYWORD_JOIN:
        mainTrack->board->power(on, false);
        progTrack->board->power(on, false);
        if(on) {
          CommManager::broadcast(F("<p1 JOIN>"));
        }
        else CommManager::broadcast(F("<p0>"));
        return;
      }
      DIAG(F("\nUnexpected keyword hash=%d\n"),p[0]);
    }
    break;

/***** READ MAIN OPERATIONS TRACK CURRENT  ****/

  case 'c':     // <c>
    // TODO(davidcutting42@gmail.com): When JMRI moves to milliamp reporting, 
    // fix this.
    int currRead;
    currRead = mainTrack->board->getCurrentRaw();
    CommManager::send(stream, F("<a %d>"), currRead);
    break;

/***** READ NUMBER OF SUPPORTED MOBILE DECODERS ****/

  case '#':     // <#>
      CommManager::send(stream, F("<# %d>"), mainTrack->numLocos());
    break;

/***** READ STATUS OF DCC++ BASE STATION  ****/

  case 's':      // <s>
    trackPowerCallback(mainTrack->board->getName(), mainTrack->board->getStatus());
    trackPowerCallback(progTrack->board->getName(), progTrack->board->getStatus());
    //  TODO(davidcutting42@gmail.com): Add throttle status notifications back
    CommManager::send(stream, 
        F("<iDCC++ CommandStation-EX / %S: V-%S / %S %S>"), 
        F(BOARD_NAME), F(VERSION), F(__DATE__), F(__TIME__));
    CommManager::showInitInfo();
    for(Turnout *tt=Turnout::firstTurnout;tt!=NULL;tt=tt->nextTurnout){
      CommManager::send(stream, F("<H %d %d %d %d>"), tt->data.id, tt->data.address, 
        tt->data.subAddress, tt->data.tStatus);
    }
    Output::show(stream);

    break;

/***** STORE SETTINGS IN EEPROM  ****/

  case 'E':     // <E>
    EEStore::store();
    CommManager::send(stream, F("<e %d %d %d>"), EEStore::eeStore->data.nTurnouts, 
      EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
    break;

/***** CLEAR SETTINGS IN EEPROM  ****/

  case 'e':     // <e>

    EEStore::clear();
    CommManager::send(stream, F("<O>"));
    break;

/***** PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW  ****/

  case ' ':     // < >
    CommManager::send(stream, F("\n"));
    break;

/***** SEND AT COMMAND TO WIFI MODULE  ****/

  case '+':
    WiFiInterface::ATCommand(com);
    break;

/***** GET THE HASH OF A STRING (FOR TRACK POWER, ETC.) ****/    

  case '$':
    if (numArgs > 1) break;
    CommManager::send(stream, F("<$ %d>"), p[0]);
    break;

/***** SET A FUNCTION USING LOCO CAB AND FUNCTION NUMBER ****/   

  case 'F':   // <F CAB NUMBER ON>
    DIAG(F("Set loco %d F%d %S"), p[0], p[1], p[2]==1 ? F("ON") : F("OFF"));
    mainTrack->setFunction(p[0], p[1], p[2]==1);
    break;

/***** FORGET A LOCOMOTIVE ****/   

  case '!':   // <F CAB NUMBER ON>
    DIAG(F("Forget loco %d"), p[0]);
    mainTrack->forgetDevice(p[0]);
    break;

  } // switch(com[0])
}

void DCCEXParser::cvResponse(Print* stream, serviceModeResponse response) {
  switch (response.type)
  {
  case READCV:
  case WRITECV:
    CommManager::send(stream, F("<r%d|%d|%d %d>"), response.callback, 
      response.callbackSub, response.cv, response.cvValue);
    break;
  case WRITECVBIT:
    CommManager::send(stream, F("<r%d|%d|%d %d %d>"), response.callback, 
      response.callbackSub, response.cv, response.cvBitNum, response.cvValue);
    break;
  }
}

void DCCEXParser::POMResponse(Print* stream, RailComPOMResponse response) {
  CommManager::send(stream, F("<k %d %x>"), response.transactionID, response.data);
}

void DCCEXParser::trackPowerCallback(const char* name, bool status) {
  if(status) 
    CommManager::broadcast(F("<p1 %s>"), name);
  else 
    CommManager::broadcast(F("<p0 %s>"), name);
}

void DCCEXParser::functionMap(int cab, uint8_t value, uint8_t fstart, uint8_t fstop) {
   for (int i = fstart; i <= fstop; i++) {
    mainTrack->setFunction(cab, i, value & 1);
    value>>=1; 
   }
}
