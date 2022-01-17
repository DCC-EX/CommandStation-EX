#ifndef version_h
#define version_h

#include "StringFormatter.h"

#define VERSION "3.2.0 rc12"
// 3.2.0 Major functional and non-functional changes.
//       New HAL added for I/O (digital and analogue inputs and outputs, servos etc).
//         Support for MCP23008, MCP23017 and PCF9584 I2C GPIO Extender modules.
//         Support for PCA9685 PWM (servo) control modules.
//         Support for analogue inputs on Arduino pins and on ADS111x I2C modules.
//         Support for MP3 sound playback via DFPlayer module.
//         Support for HC-SR04 Ultrasonic range sensor module.
//         Support for VL53L0X Laser range sensor module (Time-Of-Flight).
//       Native non-blocking I2C drivers for AVR and Nano architectures (fallback
//       to blocking Wire library for other platforms).
//       EEPROM layout change - deletes EEPROM contents on first start following upgrade.
//       New EX-RAIL automation capability.
//       Turnout class revised to expand turnout capabilities, new commands added.
//       Output class now allows ID > 255.
//       Configuration options to globally flip polarity of DCC Accessory states when driven
//       from <a> command and <T> command.
//       Increased use of display for showing loco decoder programming information.
//       Can disable EEPROM code
//       Can define border between long and short addresses
//       Turnout and accessory states (thrown/closed = 0/1 or 1/0) can be set to match RCN-213
//       Bugfix: one-off error in CIPSEND drop
//       Bugfix: disgnostic display of ack pulses >32kus
//       ...
// 3.1.7 Bugfix: Unknown locos should have speed forward 
// 3.1.6 Make output ID two bytes and guess format/size of registered outputs found in EEPROM
// 3.1.5 Fix LCD corruption on power-up
// 3.1.4 Refactor OLED and LCD drivers and remove unused code
// 3.1.3 Add a loop delay to give more time for sensing an Ethernet cable connection
// 3.1.2 Eliminate wait after write when prog is joined or prog power is off
// 3.1.1 SH1106 OLED Display Offset Fix
// 3.0.16 Ignore CV1 bit 7 read rejected by decoder when identifying loco id.  
// 3.0.15 only send function commands once, not 4 times
// 3.0.14 gap in ack tolerant fix,  prog track power management over join fix. 
// 3.0.13 Functions>127 fix
// 3.0.12 Fix HOSTNAME function for STA mode for WiFi
// 3.0.11 28 speedstep support
// 3.0.10 Teensy Support
// 3.0.9 rearranges serial newlines for the benefit of JMRI.
// 3.0.8 Includes <* *> wraps around DIAGs for the benefit of JMRI.
// 3.0.7 Includes merge from assortedBits (many changes) and ACK manager change for lazy decoders
// 3.0.6 Includes:
// Fix Bug that did not let us transmit 5 byte sized packets like PoM
// 3.0.5 Includes:
// Fix Fn Key startup with loco ID and fix state change for F16-28
// 3.0.4 Includes:
// Wifi startup bugfixes
// 3.0.3 Includes:
//  <W addr> command to write loco address and clear consist 
//  <R> command will allow for consist address
//  Startup commands implemented

#endif
