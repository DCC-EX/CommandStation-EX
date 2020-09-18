/*
 *  © 2020, Chris Harlow. All rights reserved.
 *  
 *  This file is a demonstattion of setting up a DCC-EX 
 * Command station to support direct connection of WiThrottle devices
 * such as "Engine Driver". If you contriol your layout through JMRI
 * then DON'T connect throttles to this wifi, connect them to JMRI.
 * 
 * This is just 3 statements longer than the basic setup.
 * 
 *  THIS SETUP DOES NOT APPLY TO ARDUINO UNO WITH ONLY A SINGLE SERIAL PORT.
 *  REFER TO SEPARATE EXAMPLE.        
 */

#include "DCCEX.h"

#ifdef ARDUINO_AVR_UNO 
  #include <SoftwareSerial.h>
  SoftwareSerial Serial1(15,16); // YOU must get these pins correct to use Wifi on a UNO
  #define WIFI_BAUD 9600
#else 
  #define WIFI_BAUD 115200 
#endif 

// Create a serial command parser... Enables certain diagnostics and commands
// to be issued from the USB serial console 
// This is NOT intended for JMRI....

DCCEXParser  serialParser;

void setup() {

  // The main sketch has responsibilities during setup()
  
  // Responsibility 1: Start the usb connection for diagnostics 
  // This is normally Serial but uses SerialUSB on a SAMD processor
  Serial.begin(115200);

   //  Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
   //  NOTE: References to Serial1 are for the serial port used to connect
   //        your wifi chip/shield.    
  
  Serial1.begin(WIFI_BAUD);
  WifiInterface::setup(Serial1, F("Your network name"), F("your network password"),F("DCCEX"),3532);
  
   // Responsibility 3: Start the DCC engine.
   // Note: this provides DCC with two motor drivers, main and prog, which handle the motor shield(s)
   // Standard supported devices have pre-configured macros but custome hardware installations require 
   //  detailed pin mappings and may also require modified subclasses of the MotorDriver to implement specialist logic.

   // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorShields.h

   // Optionally a Timer number (1..4) may be passed to DCC::begin to override the default Timer1 used for the
   // waveform generation.  e.g.  DCC::begin(STANDARD_MOTOR_SHIELD,2); to use timer 2
   
   DCC::begin(STANDARD_MOTOR_SHIELD);

}

void loop() {      
  // The main sketch has responsibilities during loop()
  
  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop(); 

  // Responsibility 2: handle any incoming commands on USB connection
  serialParser.loop(Serial);

  // Responsibility 3: Optionally handle any incoming WiFi traffic
  WifiInterface::loop();


}
