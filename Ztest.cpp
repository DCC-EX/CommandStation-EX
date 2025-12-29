#include <Arduino.h>
#include "DIAG.h"
#include "Ztest.h"
#include "DCCEXParser.h"
#include "StringFormatter.h"
 
RingStream * Ztest::ring = new RingStream(128);
   
void Ztest::parse(const FSH * command, const FSH * expect, bool (*test)() ) {
  
  DIAG(F("ZTEST %S"), command);

  // create copy of command in RAM
  auto commandLength= STRLEN_P((char *)command) + 1;
  char commandBuffer[commandLength+1];
  STRNCPY_P(commandBuffer, (PGM_P)command,commandLength+1);
    
  if (expect) {
    // create response buffer to collect comparison
    ring->flush();
    ring->mark(0); // mark the start of the response  
    DCCEXParser::parseOne(ring, (byte *)commandBuffer, ring);
    ring->commit();
  }
  else {
    // run without output test
    DCCEXParser::parseOne(& USB_SERIAL, (byte *)commandBuffer,nullptr);
  }

  
  // test the assert of side effects
  if (test) {
    auto result=test();
    DIAG(F("ZTEST assert %S"), result ? F("OK") : F("FAILED"));
  }

  if (expect) {
    // Copy output to serial and check with expected
    ring->read(); // read redundant client id
    auto responseLength=ring->count();
    if (responseLength < 1) {
      DIAG(F("!!....(no resp) ZTEST"));
      return; // no response
    }

    // show output while comparing
    char output[responseLength+1];
    for (int16_t i=0;i<responseLength;i++) output[i]=ring->read();   
    output[responseLength]=0;
    DIAG(F("ZTEST response:%s"), output);
    DIAG(F("ZTEST expect  :%S"), (char *)expect);
    if (STRCMP_P(output, (char *)expect) != 0)   
      DIAG(F("ZTEST expect failed\n"));

  }          
  
}
