#include "StringParser.h"
#include "DCC.h"
#include "DCCWaveform.h"
#include "DIAG.h"
// This is a JMRI command parser
// It doesnt know how the string got here, nor how it gets back.
// It knows nothing about hardware or tracks... it just parses strings and
// calls the corresponding DCC api.
//

 int StringParser::p[MAX_PARAMS];

// See documentation on DCC class for info on this section
void StringParser::parse(const char *com) {
    int params;
    bool result;
    switch(com[0]) {
    
/***** SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL ****/

    case 't':       // <t REGISTER CAB SPEED DIRECTION>
        parse(com,4);
        DCC::setThrottle(p[1],p[2],p[3]);
        DIAG(F("<T %d %d %d>"), p[0], p[2],p[3]);
        break;
    
/***** OPERATE ENGINE DECODER FUNCTIONS F0-F28 ****/

    case 'f':       // <f CAB BYTE1 [BYTE2]>
        params=parse(com,3);        
        if (params==3) DCC::setFunction(p[0],p[1],p[2]); 
        else DCC::setFunction(p[0],p[1]);

              // TODO response?
        break;

/***** OPERATE STATIONARY ACCESSORY DECODERS  ****/

    case 'a':       // <a ADDRESS SUBADDRESS ACTIVATE>
        /*
        *    turns an accessory (stationary) decoder on or off
        *
        *    ADDRESS:  the primary address of the decoder (0-511)
        *    SUBADDRESS: the subaddress of the decoder (0-3)
        *    ACTIVATE: 1=on (set), 0=off (clear)
        *
        *    Note that many decoders and controllers combine the ADDRESS and SUBADDRESS into a single number, N,
        *    from  1 through a max of 2044, where
        *
        *    N = (ADDRESS - 1) * 4 + SUBADDRESS + 1, for all ADDRESS>0
        *
        *    OR
        *
        *    ADDRESS = INT((N - 1) / 4) + 1
        *    SUBADDRESS = (N - 1) % 4
        *
        *    returns: NONE
        */
        parse(com,3);
        DCC::setAccessory(p[0],p[1],p[2]);
        break;
#ifdef THIS_IS_NOT_YET_COMPLETE    
/***** CREATE/EDIT/REMOVE/SHOW & OPERATE A TURN-OUT  ****/

    case 'T':       // <T ID THROW>
        /*
        *   <T ID THROW>:                sets turnout ID to either the "thrown" or "unthrown" position
        *
        *   ID: the numeric ID (0-32767) of the turnout to control
        *   THROW: 0 (unthrown) or 1 (thrown)
        *
        *   returns: <H ID THROW> or <X> if turnout ID does not exist
        *
        *   *** SEE ACCESSORIES.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "T" COMMAND
        *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
        */
        
        int n,s,m;
        Turnout *t;

        switch(sscanf(com+1,"%d %d %d",&n,&s,&m)){

        case 2:                     // argument is string with id number of turnout followed by zero (not thrown) or one (thrown)
            t=Turnout::get(n);
            if(t!=NULL)
                t->activate(s, (DCC*) mainTrack);
            else
                CommManager::printf("<X>");
            break;

        case 3:                     // argument is string with id number of turnout followed by an address and subAddress
            Turnout::create(n,s,m,1);
            break;

        case 1:                     // argument is a string with id number only
            Turnout::remove(n);
            break;

        case -1:                    // no arguments
            Turnout::show(1);                  // verbose show
            break;
        }
        
        break;
    
/***** CREATE/EDIT/REMOVE/SHOW & OPERATE AN OUTPUT PIN  ****/

    case 'Z':       // <Z ID ACTIVATE>
        /*
        *   <Z ID ACTIVATE>:          sets output ID to either the "active" or "inactive" state
        *
        *   ID: the numeric ID (0-32767) of the output to control
        *   ACTIVATE: 0 (active) or 1 (inactive)
        *
        *   returns: <Y ID ACTIVATE> or <X> if output ID does not exist
        *
        *   *** SEE OUTPUTS.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "O" COMMAND
        *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
        */
        
        int on,os,om;
        Output* o;

        switch(sscanf(com+1,"%d %d %d",&on,&os,&om)){

        case 2:                     // argument is string with id number of output followed by zero (LOW) or one (HIGH)
            o=Output::get(on);
            if(o!=NULL)
                o->activate(os);
            else
                CommManager::printf("<X>");
            break;

        case 3:                     // argument is string with id number of output followed by a pin number and invert flag
            Output::create(on,os,om,1);
            break;

        case 1:                     // argument is a string with id number only
            Output::remove(on);
            break;

        case -1:                    // no arguments
            Output::show(1);                  // verbose show
            break;
        }
        
        break;
    
/***** CREATE/EDIT/REMOVE/SHOW A SENSOR  ****/

    case 'S':
        
        int sn,ss,sm;

        switch(sscanf(com+1,"%d %d %d",&sn,&ss,&sm)){

        case 3:                     // argument is string with id number of sensor followed by a pin number and pullUp indicator (0=LOW/1=HIGH)
            Sensor::create(sn,ss,sm,1);
            break;

        case 1:                     // argument is a string with id number only
            Sensor::remove(sn);
            break;

        case -1:                    // no arguments
            Sensor::show();
            break;

        case 2:                     // invalid number of arguments
            CommManager::printf("<X>");
            break;
        }
    
        break;

/***** SHOW STATUS OF ALL SENSORS ****/

    case 'Q':         // <Q>
        /*
        *    returns: the status of each sensor ID in the form <Q ID> (active) or <q ID> (not active)
        */
        Sensor::status();
        break;

#endif 
/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON MAIN OPERATIONS TRACK  ****/

    case 'w':      // <w CAB CV VALUE>
        /*
        *    writes, without any verification, a Configuration Variable to the decoder of an engine on the main operations track
        *
        *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    VALUE: the value to be written to the Configuration Variable memory location (0-255)
        *
        *    returns: NONE
        */
        parse(com,3);
        DCC::writeCVByteMain(p[0],p[1],p[2]);
        break;

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON MAIN OPERATIONS TRACK  ****/

    case 'b':      // <b CAB CV BIT VALUE>
        /*
        *    writes, without any verification, a single bit within a Configuration Variable to the decoder of an engine on the main operations track
        *
        *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    BIT: the bit number of the Configurarion Variable regsiter to write (0-7)
        *    VALUE: the value of the bit to be written (0-1)
        *
        *    returns: NONE
        */
        parse(com,4);
        DCC::writeCVBitMain(p[0],p[1],p[2],p[3]);
        break;

/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON PROGRAMMING TRACK  ****/

    case 'W':      // <W CV VALUE CALLBACKNUM CALLBACKSUB>
        /*
        *    writes, and then verifies, a Configuration Variable to the decoder of an engine on the programming track
        *
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    VALUE: the value to be written to the Configuration Variable memory location (0-255)
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV Value)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if verificaiton read fails
        */
        parse(com,4);
        result=DCC::writeCVByte(p[0],p[1]);
        DIAG(F("<r%d|%d|%d %d>"), p[2], p[3],p[0],result?p[1]:-1);
        break;

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON PROGRAMMING TRACK  ****/

    case 'B':      // <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
        /*
        *    writes, and then verifies, a single bit within a Configuration Variable to the decoder of an engine on the programming track
        *
        *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
        *    BIT: the bit number of the Configurarion Variable memory location to write (0-7)
        *    VALUE: the value of the bit to be written (0-1)
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV BIT VALUE)
        *    where VALUE is a number from 0-1 as read from the requested CV bit, or -1 if verificaiton read fails
        */

        parse(com,5);
        result=DCC::writeCVBit(p[0],p[1],p[2]);
        DIAG(F("<r%d|%d|%d %d %d>"), p[3],p[4], p[0],p[1],result?p[2]:-1);
        break;

/***** READ CONFIGURATION VARIABLE BYTE FROM ENGINE DECODER ON PROGRAMMING TRACK  ****/

    case 'R':     // <R CV CALLBACKNUM CALLBACKSUB>
        /*
        *    reads a Configuration Variable from the decoder of an engine on the programming track
        *
        *    CV: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
        *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
        *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
        *
        *    returns: <r CALLBACKNUM|CALLBACKSUB|CV VALUE)
        *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
        */
        parse(com,3);
        DIAG(F("<r%d|%d|%d %d>"),p[1],p[2],p[0],DCC::readCV(p[0]));
        break;

/***** TURN ON POWER FROM MOTOR SHIELD TO TRACKS  ****/

    case '1':      // <1>
        /*
        *    enables power from the motor shield to the main operations and programming tracks
        *
        *    returns: <p1>
        */
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
        DIAG(F("<p1>"));
        break;

/***** TURN OFF POWER FROM MOTOR SHIELD TO TRACKS  ****/

    case '0':     // <0>
        /*
        *    disables power from the motor shield to the main operations and programming tracks
        *
        *    returns: <p0>
        */
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
        DIAG(F("<p0>"));
        break;

#ifdef THIS_IS_NOT_YET_COMPLETE    
/***** READ MAIN OPERATIONS TRACK CURRENT  ****/

    case 'c':     // <c>
        /*
        *    reads current being drawn on main operations track
        *
        *    returns: <a CURRENT>
        *    where CURRENT = 0-1024, based on exponentially-smoothed weighting scheme
        */
        DIAG(F("<a %d>"), DCCWaveform:mainTrack->getLastRead());
        break;

/***** READ STATUS OF DCC++ BASE STATION  ****/

    case 's':      // <s>
        /*
        *    returns status messages containing track power status, throttle status, turn-out status, and a version number
        *    NOTE: this is very useful as a first command for an interface to send to this sketch in order to verify connectivity and update any GUI to reflect actual throttle and turn-out settings
        *
        *    returns: series of status messages that can be read by an interface to determine status of DCC++ Base Station and important settings
        */
        // mainTrack->showStatus();
        for(int i=1;i<=mainTrack->numDev;i++){
            if(mainTrack->speedTable[i]==0)
            continue;
            CommManager::printf("<T%d %d %d>", i, mainTrack->speedTable[i]>0 ? mainTrack->speedTable[i] : -mainTrack->speedTable[i], mainTrack->speedTable[i]>0 ? 1 : 0);
        }
        CommManager::printf("<iDCC++ BASE STATION FOR ARDUINO %s / %s: V-%s / %s %s>", "SAMD21 Command Station", BOARD_NAME, VERSION, __DATE__, __TIME__);
        CommManager::showInitInfo();
        Turnout::show();
        Output::show();

        break;

/***** STORE SETTINGS IN EEPROM  ****/

    case 'E':     // <E>
        /*
        *    stores settings for turnouts and sensors EEPROM
        *
        *    returns: <e nTurnouts nSensors>
        */

        EEStore::store();
        CommManager::printf("<e %d %d %d>", EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        break;

/***** CLEAR SETTINGS IN EEPROM  ****/

    case 'e':     // <e>
        /*
        *    clears settings for Turnouts in EEPROM
        *
        *    returns: <O>
        */

        EEStore::clear();
        CommManager::printf("<O>");
        break;
#endif
/***** PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW  ****/

    case ' ':     // < >
        /*
        *    simply prints a carriage return - useful when interacting with Ardiuno through serial monitor window
        *
        *    returns: a carriage return
        */
        DIAG(F("\n"));
        break;
    }
}

int StringParser::parse(const char * com, byte pcount) {
    byte state=1;
  byte parameterCount=0;
  int runningValue=0;
  const char * remainingCmd=com;
  bool signNegative=false;
  while(parameterCount<pcount) {
      char hot=*remainingCmd;
       if (hot == '\0' || hot=='>') return parameterCount;
       switch (state) {
    
            case 1: // skipping spaces before a param
               if (hot==' ') break;
               continue;
               
            case 2: // checking sign
               signNegative=false;
               runningValue=0;
               if (hot=='-') {
                   signNegative=true;
                   break;
               }
               state=3;
               continue; 
            case 3: // building a parameter   
               if (hot>='0' || hot<='9') {
                   runningValue=10*runningValue+(hot-'0');
                   break;
               }
               p[parameterCount] = runningValue * (signNegative ?-1:1);
               parameterCount++;
               state=1; 
               break;
             
         }   
         remainingCmd++;
      }
      return parameterCount;
}
