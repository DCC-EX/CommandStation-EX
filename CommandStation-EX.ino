////////////////////////////////////////////////////////////////////////////////////
//  © 2020, Chris Harlow. All rights reserved.
//
//  This file is a demonstattion of setting up a DCC-EX
// Command station with optional support for direct connection of WiThrottle devices
// such as "Engine Driver". If you contriol your layout through JMRI
// then DON'T connect throttles to this wifi, connect them to JMRI.
//
//  THE WIFI FEATURE IS NOT SUPPORTED ON ARDUINO DEVICES WITH ONLY 2KB RAM.
////////////////////////////////////////////////////////////////////////////////////

#include "config.h"
#include "DCCEX.h"

// Create a serial command parser for the USB connection, 
// This supports JMRI or manual diagnostics and commands
// to be issued from the USB serial console.
DCCEXParser serialParser;

// (1) Start NetworkInterface - HTTP callback
void httpRequestHandler(ParsedRequest *req, Client* client) {
  DIAG(F("\nParsed Request:"));
  DIAG(F("\nMethod:         [%s]"), req->method);
  DIAG(F("\nURI:            [%s]"), req->uri);
  DIAG(F("\nHTTP version:   [%s]"), req->version);
  DIAG(F("\nParameter count:[%d]\n"), *req->paramCount);
}
// (1) End NetworkInterface - HTTP callback

void setup()
{
  // The main sketch has responsibilities during setup()

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  Serial.begin(115200);
  DIAG(F("DCC++ EX v%S"),F(VERSION));
   
  CONDITIONAL_LCD_START {
    // This block is ignored if LCD not in use 
    LCD(0,F("DCC++ EX v%S"),F(VERSION));
    LCD(1,F("Starting")); 
    }   

//  Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi

#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT);
#endif // WIFI_ON

  // Responsibility 3: Start the DCC engine.
  // Note: this provides DCC with two motor drivers, main and prog, which handle the motor shield(s)
  // Standard supported devices have pre-configured macros but custome hardware installations require
  //  detailed pin mappings and may also require modified subclasses of the MotorDriver to implement specialist logic.

  // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorShields.h

  // Optionally a Timer number (1..4) may be passed to DCC::begin to override the default Timer1 used for the
  // waveform generation.  e.g.  DCC::begin(STANDARD_MOTOR_SHIELD,2); to use timer 2

  DCC::begin(MOTOR_SHIELD_TYPE); 

  // (2) Start NetworkInterface - The original WifiInterface is still there but disabled

  DIAG(F("\nFree RAM before network init: [%d]\n"),freeMemory());
  DIAG(F("\nNetwork Setup In Progress ...\n"));
  NetworkInterface::setup(ETHERNET, TCP, 8888);           // specify WIFI or ETHERNET depending on if you have Wifi or an EthernetShield; Wifi has to be on Serial1 UDP or TCP for the protocol
  NetworkInterface::setHttpCallback(httpRequestHandler);  // The network interface will provide and HTTP request object which can be used as well to send the reply. cf. example above
  
  // NetworkInterface::setup(WIFI, UDP, 8888);      // Setup without port will use the by default port 2560 :: Wifi+UDP IS NOT YET SUPPORTED 
  // NetworkInterface::setup(WIFI);                 // setup without port and protocol will use by default TCP on port 2560 
  // NetworkInterface::setup();                     // all defaults ETHERNET, TCP on port 2560

  DIAG(F("\nNetwork Setup done ..."));
  DIAG(F("\nFree RAM after network init: [%d]\n"),freeMemory());

  // (2) End starting NetworkInterface

  LCD(1,F("Ready")); 
}

void loop()
{
  // The main sketch has responsibilities during loop()

  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop();

  // Responsibility 2: handle any incoming commands on USB connection
  serialParser.loop(Serial);

// Responsibility 3: Optionally handle any incoming WiFi traffic
#if WIFI_ON
  WifiInterface::loop();
#endif
// (3) Start Loop NetworkInterface
  NetworkInterface::loop();
// (3) End Loop NetworkInterface
  LCDDisplay::loop();  // ignored if LCD not in use 
  
// Optionally report any decrease in memory (will automatically trigger on first call)
#if ENABLE_FREE_MEM_WARNING
  static int ramLowWatermark = 32767; // replaced on first loop 

  int freeNow = freeMemory();
  if (freeNow < ramLowWatermark)
  {
    ramLowWatermark = freeNow;
    LCD(2,F("Free RAM=%5db"), ramLowWatermark);
  }
#endif
}
