#include "DCC.h"
#include "DIAG.h"
#include "DCCEXParser.h"
#include "WifiInterface.h"

// this code is here to demonstrate use of the DCC API and other techniques

// myFilter is an example of an OPTIONAL command filter used to intercept < > commands from
// the usb or wifi streamm.  It demonstrates how a command may be intercepted
//  or even a new command created without having to break open the API library code.
// The filter is permitted to use or modify the parameter list before passing it on to 
// the standard parser. By setting the opcode to ZERO, the standard parser will 
// just ignore the command on the assumption that you have already handled it.
//
// The filter must be enabled by calling the DCC EXParser::setFilter method, see use in setup().
 
void myFilter(Stream & stream, byte & opcode, byte & paramCount, int p[]) {
    switch (opcode) {  
       case 'F': // Invent new command to call the new Loco Function API <F cab func 1|0>
             DIAG(F("Setting loco %d F%d %S"),p[0],p[1],p[2]?F("ON"):F("OFF"));
             DCC::setFn(p[0],p[1],p[2]==1); 
             opcode=0;  // tell parser to ignore this command
             break; 
       case '#':   // Diagnose parser <#....>
            DIAG(F("# paramCount=%d\n"),paramCount);
            for (int i=0;i<paramCount;i++) DIAG(F("p[%d]=%d (0x%x)\n"),i,p[i],p[i]);
            opcode=0; // Normal parser wont understand #, 
            break;
       default:  // drop through and parser will use the command unaltered.   
            break;  
    }
}

// Callback functions are necessary if you call any API that must wait for a response from the 
// programming track. The API must return immediately otherwise other loop() functions would be blocked.
// Your callback function will be invoked when the data arrives from the prog track.

void myCallback(int result) {
  DIAG(F("\n getting Loco Id callback result=%d"),result); 
}


// Create a serial command parser... This is OPTIONAL if you don't need to handle JMRI type commands
// from the Serial port.
DCCEXParser  serialParser;


// Try monitoring the memory
#include "freeMemory.h"
int minMemory=32767;

void setup() {
   Serial.begin(SERIAL_BAUD_RATE);
   DCC::begin();
   if (WIFI_PORT>0) WifiInterface::setup();
   DIAG(F("\n===== CVReader demonstrating DCC::getLocoId() call ==========\n"));
   DCC::getLocoId(myCallback); // myCallback will be called with the result 
   DIAG(F("\n===== DCC::getLocoId has returned, but wont be executed until we are in loop() ======\n"));
   
   // Optionally tell parser to use my example filter 
   DCCEXParser::setFilter(myFilter);

   malloc(1);
   DIAG(F("\nReady for JMRI commands\n"));
   
}

void loop() {      
    DCC::loop(); // required to keep locos running and check powwer

  // This line passes input on Serial to the DCCEXParser
  serialParser.loop(Serial);

  // This line passes input on Wifi to another DCCEXParser
  if (WIFI_PORT>0) WifiInterface::loop();

  // Report any decrease in memory
  int freeNow=freeMemory();
  if (freeNow<minMemory) {
    minMemory=freeNow;
    DIAG(F("\nFree memory=%d"),minMemory);
  }
}
