#ifndef version_h
#define version_h

#include "StringFormatter.h"

#define VERSION "4.1.1"
// 4.1.1 Bugfix: preserve turnout format
//       Bugfix: parse multiple commands in one buffer string correctly (ex: <s><Q>)
//       Bugfix: </> command signal status in Exrail
//       Bugfix: EX-RAIL read long loco addr
//       Bugfix: Add space character after version string 4.1.1 for JMRI parsing.
//       Improved display and loop time for signalsmake service start to be outside the DONT_TOUCH_WIFI_CONF area
//       Improve WiFi startup by making service start to be outside the DONT_TOUCH_WIFI_CONF area
// 4.1.0 ...
//
// 4.0.2 EXRAIL additions:
//       Code: struct TurnoutData to enable EEPROM in v 4.0 format
//       Code: Fix weak reference to myFilter
//       Updated CV read command <R cv>. Equivalent to <V cv 0>.  Uses the verify callback.
//       Allow WRITE CV on PROG <W CV VALUE)
//       Change callback parameters are now optional on PROG
//       Allow turnout ID of "0"
//       Improved SIGNALs startup and diagnostics
//       Change ACK defaults set to 50mA LIMIT, 2000uS MIN, 20000uS MAX
//       myFilter automatic detection (no need to call setFilter)
//       FIX negative route ids in WIthrottle problem by allowing them 
//       Add IFRED(signal_id), IFAMBER(signal_id), IFGREEN(signal_id)
//       Add </RED signal_id> </AMBER signal_id> </GREEN signal_id> commands
//       Add <t cab> command to obtain current throttle settings 
//       Add JA, JR, JT commands to obtain route, roster and turnout descriptions
//       Add HIDDEN turnouts
//       Add PARSE <> commands in EXRAIL
//       Add VIRTUAL_TURNOUT
//       Add </KILL ALL> and KILLALL command to stop all tasks
//       Add Diagnostic messages when KILL is used
//       FORGET forgets the current loco in DCC reminder tables.
//       Add Servo signals (SERVO_SIGNAL) 
//       Add High-On signal pins (SIGNALH)
//       Add Wait for analog value (ATGTE, ATLT)
//       Allow underscore in keywords ex: MY_KEYWORD
//       Automatically assign a name with ALIAS(name) without having to define it first
//       README.md: removed misleading "folder/subforlders" (#218)
//       README.md: fix dead link to rewrite (#217) in notes/rewrite.md
//       Incoming LCN turnout throw
//       Broadcast jopin after DriveAway
//       Corrections to I2C code:
//         1) I2CManager_Mega4809.h: Correct bitwise 'and' to logical 'and' - no impact.
//         2) I2CManager_Wire.h: Ensure that error codes from Wire subsystem are passed back to caller in queueRequest().
//       Save memory on the Uno
// 4.0.1 Small EXRAIL updates
//       EXRAIL BROADCAST("msg") 
//       EXRAIL POWERON (only turns on MAIN)
//       Remove EXRAIL/ENDEXRAIL from myAutomation.example.h (#215)
//       Use "startup sequence" to describe the initial instructions
//       Add description of display scroll modes
//       restructure GetLocoCallback() for better readability and put broadcastPower() at right place
// 4.0.0 Major functional and non-functional changes.
//       Engine Driver "DriveAway" feature enhancement
//       'Discovered Server' multicast Dynamic Network Server (mDNS) displays available WiFi connections to a DCC++EX Command Station
//       New EX-RAIL "Extended Railroad Automation Instruction Language" automation capability.
//         EX-Rail Function commands for creating Automation, Route & Sequence Scripts
//         EX-RAIL “ROSTER” Engines Id & Function key layout on Engine Driver or WiThrottle
//         EX-RAIL DCC++EX Commands to Control EX-RAIL via JMRI Send pane and IDE Serial monitors
//       New JMRI feature enhancements; 
//         Reads DCC++EX EEPROM & automatically uploades any Signals, DCC Turnouts, Servo Turnouts, Vpin Turnouts , & Output pane
//         Turnout class revised to expand turnout capabilities, new commands added.
//         Provides for multiple additional DCC++EX WiFi connections as accessory controllers or CS for a programming track when Motor Shields are added
//         Supports Multiple Command Station connections and individual tracking of Send DCC++ Command panes and DCC++ Traffic Monitor panes
//       New HAL added for I/O (digital and analogue inputs and outputs, servos etc)
//         Automatically detects & connects to supported devices included in your config.h file
//         Support for MCP23008, MCP23017 and PCF9584 I2C GPIO Extender modules.
//         Support for PCA9685 PWM (servo) control modules.
//         Support for analogue inputs on Arduino pins and on ADS111x I2C modules.
//         Support for MP3 sound playback via DFPlayer module.
//         Support for HC-SR04 Ultrasonic range sensor module.
//         Support for VL53L0X Laser range sensor module (Time-Of-Flight).
//         Added <D HAL SHOW> diagnostic command to show configured devices
//       New Processor Support added
//         Compiles on Nano Every and Teensy
//       Native non-blocking I2C drivers for AVR and Nano architectures (fallback to blocking Wire library for other platforms).
//       Can disable EEPROM code
//       EEPROM layout change - deletes EEPROM contents on first start following upgrade.
//       Output class now allows ID > 255.
//       Configuration options to globally flip polarity of DCC Accessory states when driven from <a> command and <T> command.
//       Increased use of display for showing loco decoder programming information.
//       Can define border between long and short addresses
//       Turnout and accessory states (thrown/closed = 0/1 or 1/0) can be set to match RCN-213
//       Bugfix: one-off error in CIPSEND drop
//       Bugfix: disgnostic display of ack pulses >32kus
//       Bugfix: Current read from wrong ADC during interrupt
// 3.2.0 Development Release Includes all of 3.1.1 thru 3.1.7 enhancements
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
