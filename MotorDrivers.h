#ifndef MotorDrivers_h
#define MotorDrivers_h
#include <Arduino.h>

// *** PLEASE NOTE *** THIS FILE IS  **NOT**  INTENDED TO BE EDITED WHEN CONFIGURING A SYSTEM.
// It will be overwritten if the library is updated.

// This file contains configurations for known/supported motor shields.
// A configuration defined by macro here can be used in your sketch.
// A custom hardware setup will require your sketch to create MotorDriver instances
// similar to those defined here, WITHOUT editing this file. You can put your
// custom defines in config.h.

#ifndef UNUSED_PIN     // sync define with the one in MotorDriver.h
#define UNUSED_PIN 127 // inside int8_t
#endif

// MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, byte current_pin,
//             float senseFactor, unsigned int tripMilliamps, byte faultPin);
//
// If the brakePin is negative that means the sense
// of the brake pin on the motor bridge is inverted
// (HIGH == release brake)

// Arduino standard Motor Shield
#define STANDARD_MOTOR_SHIELD F("STANDARD_MOTOR_SHIELD"),                                                 \
                              new MotorDriver(3, 12, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 2000, UNUSED_PIN), \
                              new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 2000, UNUSED_PIN)

// Pololu Motor Shield
#define POLOLU_MOTOR_SHIELD F("POLOLU_MOTOR_SHIELD"),                                                 \
                            new MotorDriver( 9, 7, UNUSED_PIN,         -4, A0, 18, 3000, 12), \
                            new MotorDriver(10, 8, UNUSED_PIN, UNUSED_PIN, A1, 18, 3000, 12)
//
// Actually, on the Pololu MC33926 shield the enable lines are tied together on pin 4 and the
// pins 9 and 10 work as "inverted brake" but as we turn on and off the tracks individually
// via the power pins we above use 9 and 10 as power pins and 4 as "inverted brake" which in this
// version of the code always will be high. That means this config is not usable for generating
// a railcom cuotout in the future. For that one must wire the second ^D2 to pin 2 and define
// the motor driver like this:
//                          new MotorDriver(4, 7, UNUSED_PIN,  -9, A0, 18, 3000, 12)
//                          new MotorDriver(2, 8, UNUSED_PIN, -10, A1, 18, 3000, 12)
// See Pololu dial_mc33926_shield_schematic.pdf and truth table on page 17 of the MC33926 data sheet.

// Firebox Mk1
#define FIREBOX_MK1 F("FIREBOX_MK1"),                                                  \
                    new MotorDriver(3, 6, 7, UNUSED_PIN, A5, 9.766, 5500, UNUSED_PIN), \
                    new MotorDriver(4, 8, 9, UNUSED_PIN, A1, 5.00, 1000, UNUSED_PIN)

// Firebox Mk1S
#define FIREBOX_MK1S F("FIREBOX_MK1A"),                                            \
                     new MotorDriver(24, 21, 22, 25, 23, 9.766, 5500, UNUSED_PIN), \
                     new MotorDriver(30, 27, 28, 31, 29, 5.00, 1000, UNUSED_PIN)

// FunduMoto Motor Shield
#define FUNDUMOTO_SHIELD F("FUNDUMOTO_SHIELD"),                                              \
                         new MotorDriver(10, 12, UNUSED_PIN, 9, A0, 2.99, 2000, UNUSED_PIN), \
                         new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 2000, UNUSED_PIN)

// IBT_2 Motor Board for Main and Arduino Motor Shield for Prog
#define IBT_2_WITH_ARDUINO F("IBT_2_WITH_ARDUINO_SHIELD"),                                              \
                         new MotorDriver(4, 5, 6, UNUSED_PIN, A5, 41.54, 5000, UNUSED_PIN), \
                         new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 2000, UNUSED_PIN)

#endif
