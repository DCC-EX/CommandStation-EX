/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This is a basic, no frills CVreader example of a DCC++ compatible setup.
 *  There are more advanced examples in the examples folder i
 */

#include "DCCEX.h" 

// Create parser for <> commands coming from keyboard or JMRI on thr USB connection.
DCCEXParser  serialParser;

void setup() {
  
   // Responsibility 1: Start the usb connection for diagnostics and possible JMRI input
   Serial.begin(115200);
  
   // Responsibility 2: Start the DCC engine with information about your Motor Shield.
   // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorDriverss.h  
   DCC::begin(STANDARD_MOTOR_SHIELD);   
}

void loop() {      
  // Responsibility 1: Handle DCC background processes (loco reminders and power checks)
  DCC::loop(); 

  // Responsibility 2: handle any incoming commands on USB connection
  serialParser.loop(Serial);
}
