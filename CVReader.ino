#include "DCC.h"
#include "DIAG.h"
#include "DCCEXParser.h"

/* this code is here to test the waveform generator and reveal the issues involved in programming track operations.

    It tests the Waveform genartor and demonstrates how a DCC  API function can be simply written
    to transmit and receive DCC data on the programming track.

    Once started, it continues to operate as a DCC++ compaitible command parser
    Important... Config.h contains hardware specific confioguration settings
    that you will need to check.

*/

void myCallback(int result) {
  DIAG(F("\n Reading CV 1 callback result=%d"),result); 
}

void setup() {
  Serial.begin(115200);
  DCC::begin();

   DIAG(F("\n===== CVReader demonstrating DCC::readCV call ==========\n"));
   DCC::readCV(1,myCallback); // myCallback will be called with the result 
   DIAG(F("\n===== DCC::readCV has returned, but wont be executed until we are in loop() ======\n"));
   DIAG(F("\nReady for JMRI commands\n"));
}

void loop() {
  DCC::loop(); // required to keep locos running and check powwer

  // This line passes input on Serial to the DCCEXParser
  StringParser::loop(Serial, DCCEXParser::parse);
}
