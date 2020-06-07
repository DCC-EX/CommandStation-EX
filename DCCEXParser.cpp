#include "StringParser.h"
#include "DCCEXParser.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "Turnouts.h"
#include "Outputs.h"
#include "Sensors.h"

#include "EEStore.h"

const char VERSION[]="99.666";

// This is a JMRI command parser
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
// Non-DCC things like turnouts, pins and sensors are handled in additional JMRI interface classes. 


 
// See documentation on DCC class for info on this section
void DCCEXParser::parse(Stream  & stream,const char *com) {
    (void) EEPROM; // tell compiler not to warn thi is unused
    int p[MAX_PARAMS];  
    int params=StringParser::parse(com+1,p,MAX_PARAMS); 


    // Functions return from this switch if complete, break from switch implies error <X> to send
    switch(com[0]) {
    
    case 't':       // THROTTLE <t REGISTER CAB SPEED DIRECTION>
        DCC::setThrottle(p[1],p[2],p[3]);
        StringParser::send(stream,F("<T %d %d %d>"), p[0], p[2],p[3]);
        return;
    
    case 'f':       // FUNCTION <f CAB BYTE1 [BYTE2]>
        if (params==3) DCC::setFunction(p[0],p[1],p[2]); 
        else DCC::setFunction(p[0],p[1]);
        // NO RESPONSE
        return;
        
    case 'a':       // ACCESSORY <a ADDRESS SUBADDRESS ACTIVATE>
        DCC::setAccessory(p[0],p[1],p[2]);
        return;

    case 'T':       // TURNOUT  <T ...> 
        if (parseT(stream,params,p)) return;
        break;

    case 'Z':       // OUTPUT <Z ...> 
      if (parseZ(stream,params,p)) return; 
      break; 

    case 'S':        // SENSOR <S ...> 
      if (parseS(stream,params,p)) return; 
      break;
    
    case 'w':      // WRITE CV on MAIN <w CAB CV VALUE>
        DCC::writeCVByteMain(p[0],p[1],p[2]);
        return;

    case 'b':      // WRITE CV BIT ON MAIN <b CAB CV BIT VALUE>   
        DCC::writeCVBitMain(p[0],p[1],p[2],p[3]);
        return;

    case 'W':      // WRITE CV ON PROG <W CV VALUE CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream,p)) break;
        DCC::writeCVByte(p[0],p[1],callback_W);
        return;

    case 'B':      // WRITE CV BIT ON PROG <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream,p)) break; 
        DCC::writeCVBit(p[0],p[1],p[2],callback_B);
        return;
        
    case 'R':     // READ CV ON PROG <R CV CALLBACKNUM CALLBACKSUB>
        if (!stashCallback(stream,p)) break;
        DCC::readCV(p[0],callback_R);
        return;

    case '1':      // POWERON <1>
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
        StringParser::send(stream,F("<p1>"));
        return;

    case '0':     // POWEROFF <0>
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
        StringParser::send(stream,F("<p0>"));
        return;

    case 'c':     // READ CURRENT <c>
        StringParser::send(stream,F("<a %d>"), DCCWaveform::mainTrack.getLastCurrent());
        return;

    case 'Q':         // SENSORS <Q>
        Sensor::status(stream);
        break;

    case 's':      // <s>
        StringParser::send(stream,F("<iDCC-Asbelos BASE STATION FOR ARDUINO / %s: V-%s %s/%s\n>"), BOARD_NAME, VERSION, __DATE__, __TIME__ );
        // TODO send power status
        // TODO Send stats of  speed reminders table 
        // TODO send status of turnouts etc etc 
        return;

    case 'E':     // STORE EPROM <E>
        EEStore::store();
        StringParser::send(stream,F("<e %d %d %d>"), EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        return;

    case 'e':     // CLEAR EPROM <e>
        EEStore::clear();
        StringParser::send(stream, F("<O>"));
        return;

     case ' ':     // < >
        StringParser::send(stream,F("\n"));
        return;

     default:  //anything else will drop out to <X>
        break;
    
    } // end of opcode switch 

    // Any fallout here sends an <X>
       StringParser::send(stream, F("<X>"));
}

bool DCCEXParser::parseZ(Stream & stream, int params, int p[]){
      
        
    switch (params) {
        
        case 2:                     // <Z ID ACTIVATE>
           {
            Output *  o=Output::get(p[0]);      
            if(o==NULL) return false;
            o->activate(p[1]);
            StringParser::send(stream,F("<Y %d %d>"), p[0],p[1]);
           }
            return true;

        case 3:                     // <Z ID PIN INVERT>
            Output::create(p[0],p[1],p[2],1);
            return true;

        case 1:                     // <Z ID>
            return Output::remove(p[0]);
            
        case 0:                    // <Z>
            return Output::showAll(stream); 

         default:
             return false; 
        }
    }    


//===================================
bool DCCEXParser::parseT(Stream & stream, int params, int p[]) {
  switch(params){
        case 0:                    // <T>
            return (Turnout::showAll(stream));             break;

        case 1:                     // <T id>
            if (!Turnout::remove(p[0])) return false;
            StringParser::send(stream,F("<O>"));
            return true;

        case 2:                     // <T id 0|1>
             if (!Turnout::activate(p[0],p[1])) return false;
             Turnout::show(stream,p[0]);
            return true;

        case 3:                     // <T id addr subaddr>
            if (!Turnout::create(p[0],p[1],p[2])) return false;
            StringParser::send(stream,F("<O>"));
            return true;

        default:
            return false; // will <x>                    
        }
}

bool DCCEXParser::parseS(Stream & stream, int params, int p[]) {
     
        switch(params){

        case 3:                     // argument is string with id number of sensor followed by a pin number and pullUp indicator (0=LOW/1=HIGH)
            Sensor::create(p[0],p[1],p[2]);
            return true;

        case 1:                     // argument is a string with id number only
            if (Sensor::remove(p[0])) return true;
            break;

        case -1:                    // no arguments
            Sensor::show(stream);
            return true;

        default:                     // invalid number of arguments
            break;
        }
    return false;
}
 
int  DCCEXParser::stashP[MAX_PARAMS];
bool DCCEXParser::stashBusy=false;
Stream & DCCEXParser::stashStream=Serial;  // only to keep compiler happy

bool DCCEXParser::stashCallback(Stream & stream, int p[MAX_PARAMS]) {
       if (stashBusy) return false;
       stashBusy=true; 
      stashStream=stream;
      memcpy(stashP,p,MAX_PARAMS*sizeof(p[0]));
      return true;
 }       
 void DCCEXParser::callback_W(int result) {
        StringParser::send(stashStream,F("<r%d|%d|%d %d>"), stashP[2], stashP[3],stashP[0],result==1?stashP[1]:-1);
        stashBusy=false;
 }  
 
void DCCEXParser::callback_B(int result) {        
        StringParser::send(stashStream,F("<r%d|%d|%d %d %d>"), stashP[3],stashP[4], stashP[0],stashP[1],result==1?stashP[2]:-1);
        stashBusy=false;
}
void DCCEXParser::callback_R(int result) {        
        StringParser::send(stashStream,F("<r%d|%d|%d %d>"),stashP[1],stashP[2],stashP[0],result);
        stashBusy=false;
}
       
