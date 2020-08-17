#ifndef Config_h
#define Config_h

// *** PLEASE NOTE *** THIS FILE IS  **NOT**  INTENDED TO BE EDITED WHEN CONFIGURING A SYSTEM.
// It will be overwritten if the library is updated.

// This file contains configurations for known/supported motor shields.
// A configuration defined by macro here can be used in your sketch.
// A custom hardware setup will require your sketch to create MotorDriver instances
// similar to those defined here, WITHOUT editing this file.
   

const byte UNUSED_PIN = 255;

// MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, byte brake_pin, byte current_pin,
//             float senseFactor, unsigned int tripMilliamps, byte faultPin);
    
// Arduino standard Motor Shield
#define STANDARD_MOTOR_SHIELD     \
   new MotorDriver(3 , 12, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 2000, UNUSED_PIN),    \
   new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 250 , UNUSED_PIN) 

// Pololu Motor Shield
#define POLOLU_MOTOR_SHIELD     \
   new MotorDriver(4, 7, UNUSED_PIN, 9 , A0, 18, 2000, 12),    \
   new MotorDriver(2, 8, UNUSED_PIN, 10, A1, 18, 250 , UNUSED_PIN) 

// Allocations with memory implications..!
// Base system takes approx 900 bytes + 8 per loco. Turnouts, Sensors etc are dynamically created
 #ifdef ARDUINO_AVR_UNO 
  const byte MAX_LOCOS=20;
 #else 
  const byte MAX_LOCOS=50; 
 #endif              

#endif
