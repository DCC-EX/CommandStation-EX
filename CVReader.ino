#include "DCC.h"
#include "DIAG.h"
#include "DCCEXParser.h"
#include "WifiInterface.h"

/* this code is here to test the waveform generator and reveal the issues involved in programming track operations.

    It tests the Waveform genartor and demonstrates how a DCC  API function can be simply written
    to transmit and receive DCC data on the programming track.

    Once started, it continues to operate as a DCC++ compaitible command parser
    Important... Config.h contains hardware specific confioguration settings
    that you will need to check.

*/

void myCallback(int result) {
  DIAG(F("\n getting Loco Id callback result=%d"),result); 
}

DCCEXParser  serialParser;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
   DCC::begin();
   if (WIFI_PORT>0) WifiInterface::setup();
   DIAG(F("\n===== CVReader demonstrating DCC::getLocoId() call ==========\n"));
   DCC::getLocoId(myCallback); // myCallback will be called with the result 
   DIAG(F("\n===== DCC::getLocoId has returned, but wont be executed until we are in loop() ======\n"));
   DIAG(F("\nReady for JMRI commands\n"));
}

void loop() {      
    DCC::loop(); // required to keep locos running and check powwer

  // This line passes input on Serial to the DCCEXParser
  serialParser.loop(Serial);
  if (WIFI_PORT>0) WifiInterface::loop();
}
