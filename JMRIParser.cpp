#include "StringParser.h"
#include "JMRIParser.h"
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
void JMRIParser::parse(Stream  & stream,const char *com) {
    int p[MAX_PARAMS];  
    bool result;
    int params=StringParser::parse(com+1,p,MAX_PARAMS); 


    // Functions return from this switch if complete, break from switch implies error <X> to send
    switch(com[0]) {
    
/***** SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL ****/

    case 't':       // <t REGISTER CAB SPEED DIRECTION>
        DCC::setThrottle(p[1],p[2],p[3]);
        StringParser::send(stream,F("<T %d %d %d>"), p[0], p[2],p[3]);
        return;
    
/***** OPERATE ENGINE DECODER FUNCTIONS F0-F28 ****/

    case 'f':       // <f CAB BYTE1 [BYTE2]>
        if (params==3) DCC::setFunction(p[0],p[1],p[2]); 
        else DCC::setFunction(p[0],p[1]);

              // TODO response?
        return;

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
        DCC::setAccessory(p[0],p[1],p[2]);
        return;
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
        if (parseT(stream,params,p)) return;
        break;
        


/***** CREATE/EDIT/REMOVE/SHOW & OPERATE AN OUTPUT PIN  ****/

    case 'Z':       // <Z ID ACTIVATE>
      if (parseZ(stream,params,p)) return; 
      break; 
/***** CREATE/EDIT/REMOVE/SHOW A SENSOR  ****/

    case 'S':
        if (parseS(stream,params,p)) return; 
      break;
    
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
        
        DCC::writeCVByteMain(p[0],p[1],p[2]);
        return;

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
        
        DCC::writeCVBitMain(p[0],p[1],p[2],p[3]);
        return;

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
        
        result=DCC::writeCVByte(p[0],p[1]);
        StringParser::send(stream,F("<r%d|%d|%d %d>"), p[2], p[3],p[0],result?p[1]:-1);
        return;

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

        result=DCC::writeCVBit(p[0],p[1],p[2]);
        StringParser::send(stream,F("<r%d|%d|%d %d %d>"), p[3],p[4], p[0],p[1],result?p[2]:-1);
        return;

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
       
        StringParser::send(stream,F("<r%d|%d|%d %d>"),p[1],p[2],p[0],DCC::readCV(p[0]));
        return;

/***** TURN ON POWER FROM MOTOR SHIELD TO TRACKS  ****/

    case '1':      // <1>
        /*
        *    enables power from the motor shield to the main operations and programming tracks
        *
        *    returns: <p1>
        */
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::ON);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::ON);
        StringParser::send(stream,F("<p1>"));
        return;

/***** TURN OFF POWER FROM MOTOR SHIELD TO TRACKS  ****/

    case '0':     // <0>
        /*
        *    disables power from the motor shield to the main operations and programming tracks
        *
        *    returns: <p0>
        */
        DCCWaveform::mainTrack.setPowerMode(POWERMODE::OFF);
        DCCWaveform::progTrack.setPowerMode(POWERMODE::OFF);
        StringParser::send(stream,F("<p0>"));
        return;

/***** READ MAIN OPERATIONS TRACK CURRENT  ****/

    case 'c':     // <c>
        /*
        *    reads current being drawn on main operations track
        *
        *    returns: <a CURRENT>
        *    where CURRENT = 0-1024, based on exponentially-smoothed weighting scheme
        */
        StringParser::send(stream,F("<a %d>"), DCCWaveform::mainTrack.getLastCurrent());
        return;

/***** SHOW STATUS OF ALL SENSORS ****/

    case 'Q':         // <Q>
        /*
        *    returns: the status of each sensor ID in the form <Q ID> (active) or <q ID> (not active)
        */
        Sensor::status(stream);
        return;

/***** READ STATUS OF DCC++ BASE STATION  ****/
    case 's':      // <s>
        /*
        *    returns status messages containing track power status, throttle status, turn-out status, and a version number
        *    NOTE: this is very useful as a first command for an interface to send to this sketch in order to verify connectivity and update any GUI to reflect actual throttle and turn-out settings
        *
        *    returns: series of status messages that can be read by an interface to determine status of DCC++ Base Station and important settings
        */
        // TODO Send stats of  speed reminders table 
        
        StringParser::send(stream,F("<iDCC-EX BASE STATION FOR ARDUINO / %s: V-%s %s/%s\n>"), BOARD_NAME, VERSION, __DATE__, __TIME__ );

        // TODO send status of turnouts etc etc 
        return;

/***** STORE SETTINGS IN EEPROM  ****/

    case 'E':     // <E>
        /*
        *    stores settings for turnouts and sensors EEPROM
        *
        *    returns: <e nTurnouts nSensors>
        */

        EEStore::store();
        StringParser::send(stream,F("<e %d %d %d>"), EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs);
        return;

/***** CLEAR SETTINGS IN EEPROM  ****/

    case 'e':     // <e>
        /*
        *    clears settings for Turnouts in EEPROM
        *
        *    returns: <O>
        */

        EEStore::clear();
       StringParser::send(stream, F("<O>"));
        return;

        case ' ':     // < >
        /*
        *    simply prints a carriage return - useful when interacting with Ardiuno through stream monitor window
        *
        *    returns: a carriage return
        */
        StringParser::send(stream,F("\n"));
        return;

    
    } // end of opcode switch 

    // Any fallout here sends an <X>
       StringParser::send(stream, F("<X>"));
}






bool JMRIParser::parseZ(Stream & stream, int params, int p[]){
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
        
        switch (params) {
        
        case 2:                     // argument is string with id number of output followed by zero (LOW) or one (HIGH)
           {
            Output *  o=Output::get(p[0]);      
            if(o==NULL) return false;
            o->activate(p[1]);
            StringParser::send(stream,F("<Y %d %d>"), p[0],p[1]);
           }
            break;

        case 3:                     // argument is string with id number of output followed by a pin number and invert flag
            Output::create(p[0],p[1],p[2],1);
            break;

        case 1:                     // argument is a string with id number only
            Output::remove(p[0]);
            break;

        case 0:                    // no arguments
            Output::showAll(stream);                  // verbose show
            break;

         default:
             return false; 
        }
        return true; 
    }    


//===================================
bool JMRIParser::parseT(Stream & stream, int params, int p[]) {
  switch(params){
        case 0:                    // <T>
            Turnout::showAll(stream);                  // verbose show
            break;
        case 1:                     // <T id>
            if (!Turnout::remove(p[0])) break;
            StringParser::send(stream,F("<O>"));
            break;
        case 2:                     // <T id 0|1>
        if (!Turnout::activate(p[0],p[1])) return false;
             Turnout::show(stream,p[0]);
            break;

        case 3:                     // <T id addr subaddr>
            if (!Turnout::create(p[0],p[1],p[2])) return false;
            StringParser::send(stream,F("<O>"));
            break;
        default:
            return false; // will <x>                    
        }
       
  return true;
}

bool JMRIParser::parseS(Stream & stream, int params, int p[]) {
     
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
 


 
