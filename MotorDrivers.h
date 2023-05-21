/*
 *  © 2022-2023 Paul M. Antoine
 *  © 2021 Fred Decker
 *  © 2020-2023 Harald Barth
 *  (c) 2020 Chris Harlow. All rights reserved.
 *  (c) 2021 Fred Decker.  All rights reserved.
 *  (c) 2020 Harald Barth. All rights reserved.
 *  (c) 2020 Anthony W - Dayton. All rights reserved.
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
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
#define UNUSED_PIN 255 // inside uint8_t
#endif

// The MotorDriver definition is:
//
// MotorDriver(byte power_pin, byte signal_pin, byte signal_pin2, int8_t brake_pin, byte current_pin,
//             float senseFactor, unsigned int tripMilliamps, byte faultPin);
//
// power_pin:     Turns the board on/off. Often called ENABLE or PWM on the shield
// signal_pin:    Where the DCC signal goes in. Often called DIR on the shield
// signal_pin2:   Inverse of signal_pin. A few shields need this as well, can be replace by hardware inverter
// brake_pin:     When tuned on, brake is set - output shortened (*)
// current_pin:   Current sense voltage pin from shield to ADC
// senseFactor:   Relation between volts on current_pin and actual output current
// tripMilliamps: Short circuit trip limit in milliampere, max 32767 (32.767A)
// faultPin:      Some shields have a pin to to report a fault condition to the uCPU. High when fault occurs
//
// (*) If the brake_pin is negative that means the sense
// of the brake pin on the motor bridge is inverted
// (HIGH == release brake)

// Arduino STANDARD Motor Shield, used on different architectures:

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
// Standard Motor Shield definition for 3v3 processors (other than the ESP32)
// Setup for SAMD21 Sparkfun DEV board MUST use Arduino Motor Shield R3 (MUST be R3
// for 3v3 compatibility!!) senseFactor for 3.3v systems is 1.95 as calculated when using
// 10-bit A/D samples, and for 12-bit samples it's more like 0.488, but we probably need
// to tweak both these
#define STANDARD_MOTOR_SHIELD F("STANDARD_MOTOR_SHIELD"),                                                 \
                              new MotorDriver(3, 12, UNUSED_PIN, 9, A0, 0.488, 1500, UNUSED_PIN), \
                              new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 0.488, 1500, UNUSED_PIN)
#define SAMD_STANDARD_MOTOR_SHIELD STANDARD_MOTOR_SHIELD
#define STM32_STANDARD_MOTOR_SHIELD STANDARD_MOTOR_SHIELD

// EX 8874 based shield connected to a 3V3 system with 12-bit (4096) ADC
#define EX8874_SHIELD F("EX8874"), \
 new MotorDriver( 3, 12, UNUSED_PIN, 9, A0, 1.27, 5000, A4), \
 new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 1.27, 5000, A5)


#elif defined(ARDUINO_ARCH_ESP32)
// STANDARD shield on an ESPDUINO-32 (ESP32 in Uno form factor). The shield must be eiter the
// 3.3V compatible R3 version or it has to be modified to not supply more than 3.3V to the
// analog inputs. Here we use analog inputs A2 and A3 as A0 and A1 are wired in a way so that
// they are not useable at the same time as WiFi (what a bummer). The numbers below are the
// actual GPIO numbers. In comments the numbers the pins have on an Uno.
#define STANDARD_MOTOR_SHIELD F("STANDARD_MOTOR_SHIELD"), \
 new MotorDriver(25/* 3*/, 19/*12*/, UNUSED_PIN, 13/*9*/, 35/*A2*/, 0.70, 1500, UNUSED_PIN), \
 new MotorDriver(23/*11*/, 18/*13*/, UNUSED_PIN, 12/*8*/, 34/*A3*/, 0.70, 1500, UNUSED_PIN)

// EX 8874 based shield connected to a 3.3V system (like ESP32) and 12bit (4096) ADC
// numbers are GPIO numbers. comments are UNO form factor shield pin numbers
#define EX8874_SHIELD F("EX8874"),\
 new MotorDriver(25/* 3*/, 19/*12*/, UNUSED_PIN, 13/*9*/, 35/*A2*/, 1.27, 5000, 36 /*A4*/), \
 new MotorDriver(23/*11*/, 18/*13*/, UNUSED_PIN, 12/*8*/, 34/*A3*/, 1.27, 5000, 39 /*A5*/)

#else
// STANDARD shield on any Arduino Uno or Mega compatible with the original specification.
#define STANDARD_MOTOR_SHIELD F("STANDARD_MOTOR_SHIELD"),                                                 \
                              new MotorDriver(3, 12, UNUSED_PIN, 9, A0, 2.99, 1500, UNUSED_PIN), \
                              new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 2.99, 1500, UNUSED_PIN)
#define BRAKE_PWM_SWAPPED_MOTOR_SHIELD F("BPS_MOTOR_SHIELD"),                                       \
                              new MotorDriver(-9 , 12, UNUSED_PIN, -3, A0, 2.99, 1500, UNUSED_PIN), \
                              new MotorDriver(-8 , 13, UNUSED_PIN,-11, A1, 2.99, 1500, UNUSED_PIN)

// EX 8874 based shield connected to a 5V system (like Arduino) and 10bit (1024) ADC
#define EX8874_SHIELD F("EX8874"), \
 new MotorDriver( 3, 12, UNUSED_PIN, 9, A0, 5.08, 5000, A4), \
 new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 5.08, 5000, A5)

#endif

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

// Pololu Dual TB9051FTG Motor Shield
// This is the shield without modifications. Unfortunately the TB9051FTG driver chip on
// the shield makes short delays when direction is switched. That means that the chip
// can NOT provide a standard conformant DCC signal independent how hard we try. If your
// Decoders tolerate that signal, use it by all mean but it is not recommended. Without
// modifications it uses the following pins below which means no HA waveform and no
// RailCom on an Arduino Mega 2560 but the DCC signal is broken anyway.
#define POLOLU_TB9051FTG F("POLOLU_TB9051FTG"),              \
   new MotorDriver(2, 7, UNUSED_PIN,  -9, A0, 10, 2500,  6), \
   new MotorDriver(4, 8, UNUSED_PIN, -10, A1, 10, 2500, 12)

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
                         new MotorDriver(10, 12, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 1500, UNUSED_PIN), \
                         new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 1500, UNUSED_PIN)

// IBT_2 Motor Board for Main and Arduino Motor Shield for Prog
#define IBT_2_WITH_ARDUINO F("IBT_2_WITH_ARDUINO_SHIELD"),                                              \
                         new MotorDriver(4, 5, 6, UNUSED_PIN, A5, 41.54, 5000, UNUSED_PIN), \
                         new MotorDriver(11, 13, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 1500, UNUSED_PIN)
// YFROBOT Motor Shield (V3.1)
#define YFROBOT_MOTOR_SHIELD F("YFROBOT_MOTOR_SHIELD"), \
    new MotorDriver(5, 4, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 1500, UNUSED_PIN), \
    new MotorDriver(6, 7, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 1500, UNUSED_PIN)

// Makeblock ORION UNO like sized board with integrated motor driver
// This is like an Uno with H-bridge and RJ12 contacts instead of pin rows.
// No current sense. Barrel connector max 12V, Vmotor max 15V. 1.1A polyfuse as output protection.
// Main is marked M1 and near RJ12 #5
// Prog is marked M2 and near RJ12 #4
// For details see
// http://docs.makeblock.com/diy-platform/en/electronic-modules/main-control-boards/makeblock-orion.html
#define ORION_UNO_INTEGRATED_SHIELD F("ORION_UNO_INTEGRATED_SHIELD"),		      \
    new MotorDriver(6, 7, UNUSED_PIN, UNUSED_PIN, UNUSED_PIN, 1.0, 1100, UNUSED_PIN), \
    new MotorDriver(5, 4, UNUSED_PIN, UNUSED_PIN, UNUSED_PIN, 1.0, 1100, UNUSED_PIN)

// This is an example how to setup a motor shield definition for a motor shield connected
// to an NANO EVERY board. You have to make the connectons from the shield to the board
// as in this example or adjust the values yourself.
#define NANOEVERY_EXAMPLE F("NANOEVERY_EXAMPLE"), \
 new MotorDriver(5,  6, UNUSED_PIN, UNUSED_PIN, A0, 2.99, 1500, UNUSED_PIN),\
 new MotorDriver(9, 10, UNUSED_PIN, UNUSED_PIN, A1, 2.99, 1500, UNUSED_PIN)

// This is an example how to stack two standard motor shields. The upper shield
// needs pins 3 8 9 11 12 13 A0 A1 disconnected from the lower shield and
// jumpered instead like this:  2-3 6-8 7-9 4-13 5-11 10-12 A0-A4 A1-A5
// Pin assigment table:
// 2 Enable C  jumpered
// 3 Enable A  direct
// 4 Dir D     jumpered
// 5 Enable D  jumpered
// 6 Brake D   jumpered
// 7 Brake C   jumpered
// 8 Brake B   direct
// 9 Brake A   direct
// 10 Dir C    jumpered
// 11 Enable B direct
// 12 Dir A    direct
// 13 Dir B    direct
// A0 Sense A  direct
// A1 Sense B  direct
// A4 Sense C  jumpered
// A5 Sense D  jumpered
//
#define STACKED_MOTOR_SHIELD F("STACKED_MOTOR_SHIELD"),\
  new MotorDriver( 3, 12, UNUSED_PIN, 9, A0, 2.99, 1500, UNUSED_PIN), \
  new MotorDriver(11, 13, UNUSED_PIN, 8, A1, 2.99, 1500, UNUSED_PIN), \
  new MotorDriver( 2, 10, UNUSED_PIN, 7, A4, 2.99, 1500, UNUSED_PIN), \
  new MotorDriver( 5,  4, UNUSED_PIN, 6, A5, 2.99, 1500, UNUSED_PIN)
//
#endif
