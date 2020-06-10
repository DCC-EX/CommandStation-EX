#include "StringFormatter.h"
#include "DCCEXParser.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "Turnouts.h"
#include "Outputs.h"
#include "Sensors.h"

#include "EEStore.h"

const char VERSION[]="99.666";

int DCCEXParser::stashP[MAX_PARAMS];
 bool DCCEXParser::stashBusy;
     Stream & DCCEXParser::stashStream=Serial;  // keep compiler happy but ovevride in constructor


DCCEXParser::DCCEXParser(Stream & myStream) {
   stream=myStream;
}

// This is a JMRI command parser, one instance per incoming stream
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
// Non-DCC things like turnouts, pins and sensors are handled in additional JMRI interface classes. 

void DCCEXParser::loop() {
  while(stream.available()) {
    if (bufferLength==MAX_BUFFER) {
       bufferLength=0;
      inCommandPayload=false;
    }
    char ch = stream.read();
    if (ch == '<') {
      inCommandPayload = true;
      bufferLength=0;
      buffer[0]='\0';
    } 
    else if (ch == '>') {
      buffer[bufferLength]='\0';
      parse(buffer);
      inCommandPayload = false;
      break;
    } else if(inCommandPayload) {
      buffer[bufferLength++]= ch;
    }
  }
  }

 int DCCEXParser::splitValues( int result[MAX_PARAMS]) {
  byte state=1;
  byte parameterCount=0;
  int runningValue=0;
  const char * remainingCmd=buffer+1;  // skips the opcode
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
void DCCEXParser::parse(const char *com) {
    (void) EEPROM; // tell compiler not to warn thi is unused
    int p[MAX_PARAMS];  
    int params=splitValues(p); 


    // Functions return from this switch if complete, break from switch implies error <X> to send
    switch(com[0]) {
    
    case 't':       // THROTTLE <t REGISTER CAB SPEED DIRECTION>
        DCC::setThrottle(p[1],p[2],p[3]);
        StringFormatter::send(stream,F("<T %d %d %d>"), p[0], p[2],p[3]);
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
        if (parseT(params,p)) return;
        break;

    case 'Z':       // OUTPUT <Z ...> 
      if (parseZ(params,p)) return; 
      break; 

    case 'S':        // SENSOR <S ...> 
      if (parseS(params,p)) return; 
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
        StringFormatter::send(stream,F("<p1>"));
        return;

    case '0':     // POWEROFF <0>
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
        StringFormatter::send(stream,F("<p0>"));
        return;

    case 'c':     // READ CURRENT <c>
        StringFormatter::send(stream,F("<a %d>"), DCCWaveform::mainTrack.getLastCurrent());
        return;

    case 'Q':         // SENSORS <Q>
        Sensor::status(stream);
        break;

    case 's':      // <s>
        StringFormatter::send(stream,F("<iDCC-Asbelos BASE STATION FOR ARDUINO / %s: V-%s %s/%s\n>"), BOARD_NAME, VERSION, __DATE__, __TIME__ );
        // TODO send power status
        // TODO Send stats of  speed reminders table 
        // TODO send status of turnouts etc etc 
        return;

    case 'E':     // STORE EPROM <E>
        EEStore::store();
        StringFormatter::send(stream,F("<e %d %d %d>"), EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        return;

    case 'e':     // CLEAR EPROM <e>
        EEStore::clear();
        StringFormatter::send(stream, F("<O>"));
        return;

     case ' ':     // < >
        StringFormatter::send(stream,F("\n"));
        return;

     default:  //anything else will drop out to <X>
        break;
    
    } // end of opcode switch 

    // Any fallout here sends an <X>
       StringFormatter::send(stream, F("<X>"));
}

bool DCCEXParser::parseZ( int params, int p[]){
      
        
    switch (params) {
        
        case 2:                     // <Z ID ACTIVATE>
           {
            Output *  o=Output::get(p[0]);      
            if(o==NULL) return false;
            o->activate(p[1]);
            StringFormatter::send(stream,F("<Y %d %d>"), p[0],p[1]);
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
bool DCCEXParser::parseT( int params, int p[]) {
  switch(params){
        case 0:                    // <T>
            return (Turnout::showAll(stream));             break;

        case 1:                     // <T id>
            if (!Turnout::remove(p[0])) return false;
            StringFormatter::send(stream,F("<O>"));
            return true;

        case 2:                     // <T id 0|1>
             if (!Turnout::activate(p[0],p[1])) return false;
             Turnout::show(stream,p[0]);
            return true;

        case 3:                     // <T id addr subaddr>
            if (!Turnout::create(p[0],p[1],p[2])) return false;
            StringFormatter::send(stream,F("<O>"));
            return true;

        default:
            return false; // will <x>                    
        }
}

bool DCCEXParser::parseS( int params, int p[]) {
     
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


 // CALLBACKS must be static 
bool DCCEXParser::stashCallback(Stream & stream, int p[MAX_PARAMS]) {
       if (stashBusy) return false;
       stashBusy=true; 
      stashStream=stream;
      memcpy(stashP,p,MAX_PARAMS*sizeof(p[0]));
      return true;
 }       
 void DCCEXParser::callback_W(int result) {
        StringFormatter::send(stashStream,F("<r%d|%d|%d %d>"), stashP[2], stashP[3],stashP[0],result==1?stashP[1]:-1);
        stashBusy=false;
 }  
 
void DCCEXParser::callback_B(int result) {        
        StringFormatter::send(stashStream,F("<r%d|%d|%d %d %d>"), stashP[3],stashP[4], stashP[0],stashP[1],result==1?stashP[2]:-1);
        stashBusy=false;
}
void DCCEXParser::callback_R(int result) {        
        StringFormatter::send(stashStream,F("<r%d|%d|%d %d>"),stashP[1],stashP[2],stashP[0],result);
        stashBusy=false;
}
       
