#include "DCC.h"
#include "DIAG.h"
#include "DCCEXParser.h"

/* this code is here to test the waveforwe generator and reveal the issues involved in programming track operations.

    It tests the Waveform genartor and demonstrates how a DCC  API function can be simply written
    to transmit and receive DCC data on the programming track.

    Once soem CVs have been listed, it then drops into JMRI input moce so you can play.
    
    Important... Config.h contains hardware specific confioguration settings
    that you will need to check.



*/



const int cvnums[] = {1, 2, 3, 4, 5, 8, 17, 18, 19, 21, 22, 29};

void setup() {
  Serial.begin(115200);
  DCC::begin();

//  DIAG(F("\n===== CVReader begin ==============================\n"));
//
//  for (byte x = 0; x < sizeof(cvnums) / sizeof(cvnums[0]); x++) {
//    int value = DCC::readCV(cvnums[x]);
//    DIAG(F("\nCV %d = %d  0x%x  %s\n"), cvnums[x], value, value, value >= 0 ? " VERIFIED OK" : "FAILED VERIFICATION");
//  }
//  DIAG(F("\n===== CVReader done ==============================\n"));
  DIAG(F("\nReady for JMRI commands\n"));
}

void loop() {
  DCC::loop(); // required to keep locos running and check powwer

  // This line passes input on Serial to the DCCEXParser
  StringParser::loop(Serial, DCCEXParser::parse);
}
