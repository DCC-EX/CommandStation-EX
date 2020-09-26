#ifndef MotorDrivers_h
#define MotorDrivers_h
#if defined(ARDUINO_ARCH_MEGAAVR)
#include <Arduino.h>
#endif

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
#define STANDARD_MOTOR_SHIELD F("STANDARD_MOTOR_SHIELD"),                                                 \
                              new MotorDriver(3, 12, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 2000, UNUSED_PIN), \
                              new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, TRIP_CURRENT_PROG, UNUSED_PIN)

// Pololu Motor Shield
#define POLOLU_MOTOR_SHIELD F("POLOLU_MOTOR_SHIELD"),                               \
                            new MotorDriver(4, 7, UNUSED_PIN, 9, A0, 18, 2000, 12), \
                            new MotorDriver(2, 8, UNUSED_PIN, 10, A1, 18, TRIP_CURRENT_PROG, UNUSED_PIN)

// Firebox Mk1
#define FIREBOX_MK1 F("FIREBOX_MK1"),                                                  \
                    new MotorDriver(3, 6, 7, UNUSED_PIN, A5, 9.766, 5500, UNUSED_PIN), \
                    new MotorDriver(4, 8, 9, UNUSED_PIN, A1, 5.00, TRIP_CURRENT_PROG, UNUSED_PIN)

// Firebox Mk1S
#define FIREBOX_MK1S F("FIREBOX_MK1A"),                                            \
                     new MotorDriver(24, 21, 22, 25, 23, 9.766, 5500, UNUSED_PIN), \
                     new MotorDriver(30, 27, 28, 31, 29, 5.00, TRIP_CURRENT_PROG, UNUSED_PIN)

// FunduMoto Motor Shield
#define FUNDUMOTO_SHIELD F("FUNDUMOTO_SHIELD"),                                              \
                         new MotorDriver(10, 12, UNUSED_PIN, 9, A0, 2.99, 2000, UNUSED_PIN), \
                         new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, TRIP_CURRENT_PROG, UNUSED_PIN)

#endif
