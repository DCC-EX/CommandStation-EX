#ifndef Config_h
#define Config_h

// *** PLEASE NOTE *** THIS FILE IS  **NOT**  INTENDED TO BE EDITED WHEN CONFIGURING A SYSTEM.
// It will be overwritten if the library is updated.

// This file contains configurations for known/supported motor shields.
// A configuration defined by macro here can be used in your sketch.
// A custom hardware setup will require your sketch to create MotorDriver instances
// similar to those defined here, WITHOUT editing this file.
   
// Define these if you have a WiFi board on Serial1
#define WIFI
#define WIFI_CONNECT_TO_SSID  "RPi-JMRI"
#define WIFI_CONNECT_PASSWORD "rpI-jmri"

const byte UNUSED_PIN = 255;

// MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, byte brake_pin, byte current_pin, float senseFactor, unsigned int tripMilliamps, byte faultPin);
    
// Arduino standard Motor Shield
#define STANDARD_MOTOR_SHIELD     \
   new MotorDriver(3 , 12, UNUSED_PIN, 9, A0, 2.99, 2000, UNUSED_PIN),    \
   new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 2.99, 250 , UNUSED_PIN) 


// Allocations with memory implications..!
// Base system takes approx 900 bytes + 8 per loco. Turnouts, Sensors etc are dynamically created
const byte MAX_LOCOS=50;             

#endif
