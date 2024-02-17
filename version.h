#ifndef version_h
#define version_h

#include "StringFormatter.h"

#define VERSION "5.2.35"
// 5.2.35 - Bugfix: Make DCC Extended Accessories follow RCN-213
// 5.2.34 - <A address aspect> Command fopr DCC Extended Accessories
//        - Exrail ASPECT(address,aspect) for above.
//        - EXRAIL DCCX_SIGNAL(Address,redAspect,amberAspect,greenAspect)
//        - Exrail intercept <A ...> for DCC Signals. 
// 5.2.33 - Exrail CONFIGURE_SERVO(vpin,pos1,pos2,profile)
// 5.2.32 - Railcom Cutout (Initial trial Mega2560 only)
// 5.2.31 - Exrail JMRI_SENSOR(vpin [,count]) creates <S> types.  
// 5.2.30 - Bugfix: WiThrottle sendIntro after initial N message as well
// 5.2.29 - Added IO_I2CDFPlayer.h to support DFPLayer over I2C connected to NXP SC16IS750/SC16IS752 (currently only single UART for SC16IS752)
//        - Added enhanced IO_I2CDFPLayer enum commands to EXRAIL2.h
//        - Added PLAYSOUND alias of ANOUT to EXRAILMacros.h
//        - Added UART detection to I2CManager.cpp
// 5.2.28 - ESP32: Can all Wifi channels.
//        - ESP32: Only write Wifi password to display if it is a well known one
// 5.2.27 - Bugfix: IOExpander memory allocation
// 5.2.26 - Silently ignore overridden HAL defaults
//        - include HAL_IGNORE_DEFAULTS macro in EXRAIL 
// 5.2.25 - Fix bug causing <X> after working <D commands 
// 5.2.24 - Exrail macro asserts to catch 
//            : duplicate/missing automation/route/sequence/call ids
//            : latches and reserves out of range
//            : speeds out of range
// 5.2.23 - KeywordHasher _hk (no functional change) 
// 5.2.22 - Bugfixes: Empty turnout descriptions ok; negative route numbers valid.
// 5.2.21 - Add STARTUP_DELAY config option to delay CS bootup
// 5.2.20 - Check return of Ethernet.begin()
// 5.2.19 - ESP32: Determine if the RMT hardware can handle DCC
// 5.2.18 - Display network IP fix
// 5.2.17 - ESP32 simplify network logic
// 5.2.16 - Bugfix to allow for devices using the EX-IOExpander protocol to have no analogue or no digital pins
// 5.2.15 - move call to CommandDistributor::broadcastPower() into the TrackManager::setTrackPower(*) functions
//        - add repeats to function packets that are not reminded in accordance with accessory packets
// 5.2.14 - Reminder window DCC packet optimization
//        - Optional #define DISABLE_FUNCTION_REMINDERS 
// 5.2.13 - EXRAIL STEALTH 
// 5.2.12 - ESP32 add AP mode LCD messages with SSID/PW for
//        - STM32 change to UID_BASE constants in DCCTimerSTM32 rather than raw hex addresses for UID registers
//        - STM32 extra UART/USARTs for larger Nucleo models
// 5.2.11 - Change from TrackManager::returnMode to TrackManager::getMode
// 5.2.10 - Include trainbrains.eu block unoccupancy driver
//        - include IO_PCA9555  
// 5.2.9  - Bugfix LCD startup with no LCD, uses <@
// 5.2.9  - EXRAIL STASH feature 
// 5.2.8  - Bugfix: Do not turn off all tracks on change
//          give better power messages
// 5.2.7  - Bugfix: EXRAIL ling segment
//        - Bugfix: Back out wrongly added const
//        - Bugfix ESP32: Do not inverse DCX direction signal twice
// 5.2.6  - Trackmanager broadcast power state on track mode change
// 5.2.5  - Trackmanager: Do not treat TRACK_MODE_ALL as TRACK_MODE_DC
// 5.2.4  - LCD macro will not do diag if that duplicates @ to same target.
//        - Added ROUTE_DISABLED macro in EXRAIL
// 5.2.3  - Bugfix: Catch stange input to parser
// 5.2.2  - Added option to allow MAX_CHARACTER_ROWS to be defined in config.h
// 5.2.1  - Trackmanager rework for simpler structure
// 5.2.0  - ESP32: Autoreverse and booster mode support
// 5.1.21 - EXRAIL invoke multiple ON handlers for same event
// 5.1.20 - EXRAIL Tidy and ROUTE_STATE, ROUTE_CAPTION 
// 5.1.19 - Only flag 2.2.0.0-dev as broken, not 2.2.0.0
// 5.1.18 - TURNOUTL bugfix
// 5.1.17 - Divide out C for config and D for diag commands
// 5.1.16 - Remove I2C address from EXTT_TURNTABLE macro to work with MUX, requires separate HAL macro to create
// 5.1.15 - LCC/Adapter support and Exrail feature-compile-out.
// 5.1.14 - Fixed IFTTPOSITION
// 5.1.13 - Changed turntable broadcast from i to I due to server string conflict
// 5.1.12 - Added Power commands <0 A> & <1 A> etc. and update to <=>
//          Added EXRAIL SET_POWER(track, ON/OFF)
//          Fixed a problem whereby <1 MAIN> also powered on PROG track
//          Added functions to TrackManager.cpp to allow UserAddin code for power display on OLED/LCD
//          Added - returnMode(byte t), returnDCAddr(byte t) & getModeName(byte Mode)
// 5.1.11 - STM32F4xx revised I2C clock setup, no correctly sets clock and has fully variable frequency selection
// 5.1.10 - STM32F4xx DCCEXanalogWrite to handle PWM generation for TrackManager DC/DCX
//        - STM32F4xx DCC 58uS timer now using non-PWM output timers where possible
//        - ESP32 brakeCanPWM check now detects UNUSED_PIN
//        - ARM architecture brakeCanPWM now uses digitalPinHasPWM()
//        - STM32F4xx shadowpin extensions to handle pins on ports D, E and F
// 5.1.9  - Fixed IO_PCA9555'h to work with PCA9548 mux, tested OK
// 5.1.8  - STM32Fxx ADCee extension to support ADCs #2 and #3
// 5.1.7  - Fix turntable broadcasts for non-movement activities and <JP> result
// 5.1.6  - STM32F4xx native I2C driver added
// 5.1.5  - Added turntable object and EXRAIL commands
//        - <I ...>, <JO ...>, <JP ...> - turntable commands
//        - DCC_TURNTABLE, EXTT_TURNTABLE, IFTTPOSITION, ONROTATE, ROTATE, ROTATE_DCC, TT_ADDPOSITION, WAITFORTT EXRAIL
// 5.1.4  - Added ONOVERLOAD & AFTEROVERLOAD to EXRAIL
// 5.1.3  - Make parser more fool proof
// 5.1.2  - Bugfix: ESP32 30ms off time
// 5.1.1  - Check bad AT firmware version
//        - Update IO_PCA9555.h reflecting IO_MCP23017.h changes to support PCA9548 mux
// 5.0.1  - Bugfix: execute 30ms off time before rejoin
// 5.0.0  - Make 4.2.69 the 5.0.0 release
// 4.2.69 - Bugfix: Make <!> work in DC mode
// 4.2.68 - Rename track mode OFF to NONE
// 4.2.67 - AVR: Pin specific timer register seting
//        - Protect Uno user from choosing DC(X)
//        - More Nucleo variant defines
//        - GPIO PCA9555 / TCA9555 support
// 4.2.66 - Throttle inrush current by applying PWM to brake pin when
//          fault pin goes active
// 4.2.65 - new config WIFI_FORCE_AP option
// 4.2.63 - completely new overcurrent detection
//        - ESP32 protect from race in RMT code
// 4.2.62 - Update IO_RotaryEncoder.h to ignore sending current position
//        - Update IO_EXTurntable.h to remove forced I2C clock speed
//        - Show device offline if EX-Turntable not connected
// 4.2.61 - MAX_CURRENT restriction (caps motor shield value)
// 4.2.60 - Add mDNS capability to ESP32 for autodiscovery
// 4.2.59 - Fix: AP SSID was DCC_ instead of DCCEX_
// 4.2.58 - Start motordriver as soon as possible but without waveform
// 4.2.57 - New overload handling (faster and handles commonFaultPin again)
//        - Optimize analog read STM32
// 4.2.56 - Update IO_RotaryEncoder.h:
//        - Improved I2C communication, non-blocking reads
//        - Enable sending positions to the encoder from EXRAIL via SERVO()
// 4.2.55 - Optimize analog read for AVR
// 4.2.54 - EX8874 shield in config.example.h
//        - Fix: Better warnings for pin number errors
//        - Fix: Default roster list possible in Withrottle and <jR>
//        - Fix: Pin handling supports pins up to 254
// 4.2.53 - Fix: Fault pin handling made more straight forward
// 4.2.52 - Experimental support for sabertooth motor controller on ESP32
// 4.2.51 - Add DISABLE_PROG to disable programming to save RAM/Flash
// 4.2.50 - Fixes: estop all, turnout eeprom, cab ID check
// 4.2.49 - Exrail SPEED take notice of external direction change 
// 4.2.48 - BROADCAST/WITHROTTLE Exrail macros 
// 4.2.47 - Correct response to <JA 0>
// 4.2.46 - Support boards with inverted fault pin
// 4.2.45 - Add ONCLOCKMINS to FastClock to allow hourly repeat events
// 4.2.44 - Add PowerShell installer EX-CommandStation-installer.exe
// 4.2.43 - Fix STM32 set right port mode bits for analog
// 4.2.42 - Added EXRAIL TURNOUTL Macro definition
// 4.2.41 - Move HAl startup to ASAP in setup()
//        - Fix DNOU8 output pin setup to all LOW  
// 4.2.40 - Automatically detect conflicting default I2C devices and disable
// 4.2.39 - DFplayer driver now polls device to detect failures and errors.
// 4.2.38 - Clean up compiler warning when IO_RotaryEncoder.h included
// 4.2.37 - Add new FLAGS HAL device for communications to/from EX-RAIL;
//        - Fix diag display of high VPINs within IODevice class.
// 4.2.36 - do not broadcast a turnout state that has not changed
//        - Use A2/A3 for current sensing on ESP32 + Motor Shield
// 4.2.35 - add <z> direct pin manipulation command 
// 4.2.34 - Completely fix EX-IOExpander analogue inputs
// 4.2.33 - Fix EX-IOExpander non-working analogue inputs
// 4.2.32 - Fix LCD/Display bugfixes from 4.2.29
// 4.2.31 - Removes EXRAIL statup from top of file. (BREAKING CHANGE !!)
//          Just add AUTOSTART to the top of your myAutomation.h to restore this function.
// 4.2.30 - Fixes/enhancements to EX-IOExpander device driver.
// 4.2.29 - Bugfix Scroll LCD without empty lines and consistent
// 4.2.28 - Reinstate use of timer11 in STM32 - remove HA mode.
//        - Update IO_DFPlayer to work with MP3-TF-16P rev3.
// 4.2.27 - Bugfix LCD showed random characters in SCROLLMODE 2
// 4.2.26 - EX-IOExpander device driver enhancements
//        - Enhance I2C error checking
//        - Introduce delays to _loop to allow room for other I2C device comms
//        - Improve analogue read reliability
// 4.2.25 - Bugfix SAMD21 Exrail odd byte boundary
// 4.2.24 - Bugfix Ethernet shield: Static IP now possible
// 4.2.23 - Bugfix signalpin2 was not set up in shadow port
// 4.2.22 - Implement broadcast of Track Manager changes
// 4.2.21 - Implement non-blocking I2C for EX-IOExpander device driver
// 4.2.20 - <JG> & <JI> commands for multi-track gauges
//        - Reinstate <c> but remember its a bit useless when TM involved.   
// 4.2.19 - Bugfix for analog reading of track current sensor offset.
// 4.2.18 - I2C Multiplexer support through Extended Addresses,
//          added for Wire, 4209 and AVR I2C drivers.
//        - I2C retries when an operation fails.
//        - I2C timeout handling and recovery completed.
//        - I2C SAMD Driver Read code completed.
//        - PCF8575 I2C GPIO driver added.
//        - EX-RAIL ANOUT function for triggering analogue
//          HAL drivers (e.g. analogue outputs, DFPlayer, PWM).
//        - EX-RAIL SCREEN function for writing to screens other 
//          than the primary one.
//        - Installable HALDisplay Driver, with support
//          for multiple displays.
//        - Layered HAL Drivers PCA9685pwm and Servo added for 
//          native PWM on PCA9685 module and
//          for animations of servo movement via PCA9685pwm.
//          This is intended to support EXIOExpander and also
//          replace the existing PCA9685 driver.
//        - Add <D HAL RESET> to reinitialise failed drivers.
//        - Add UserAddin facility to allow a user-written C++ function to be 
//          declared in myHal.cpp, to be called at a user-specified frequency.
//        - Add ability to configure clock speed of PCA9685 drivers 
//          (to allow flicker-free LED control).
//        - Improve stability of VL53L0X driver when XSHUT pin connected.
//        - Enable DCC high accuracy mode for STM32 on standard motor shield (pins D12/D13).
//        - Incorporate improvements to ADC scanning performance (courtesy of HABA).
// 4.2.17 LCN bugfix
// 4.2.16 Move EX-IOExpander servo support to the EX-IOExpander software
// 4.2.15 Add basic experimental PWM support to EX-IOExpander
//        EX-IOExpander 0.0.14 minimum required
// 4.2.14 STM32F4xx fast ADC read implementation
// 4.2.13 Broadcast power for <s> again
// 4.2.12 Bugfix for issue #299 TurnoutDescription NULL
// 4.2.11 Exrail IFLOCO feature added
// 4.2.10 SIGNAL/SIGNALH bug fix as they were inverted
//        IO_EXIOExpander.h input speed optimisation
//        ONCLOCK and ONCLOCKTIME command added to EXRAIL for EX-FastCLock
//        <JC> Serial command added for EX-FastClock
//        <jC> Broadcast added for EX-FastClock
//        IO_EXFastClock.h added for I2C FastClock connection
// 4.2.9 duinoNodes support
// 4.2.8 HIGHMEM (EXRAIL support beyond 64kb)
//       Withrottle connect/disconnect improvements
//       Report BOARD_TYPE if provided by compiler
// 4.2.7 FIX: Static IP addr
//       FIX: Reuse WiThrottle list entries
// 4.2.6 FIX: Remove RAM thief
//       FIX: ADC port 8-15 fix
// 4.2.5 Make GETFLASHW code more universal
//       FIX: Withrottle roster index
//       Ethernet start improvement and link detection
// 4.2.4 ESP32 experimental BT support
//       More DC configurations possible and lower frequency
//       Handle decoders that do not ack at write better
// 4.2.3 Bugfix direction when togging between MAIN and DC
//       Bugfix return fail when F/f argument out of range
//       More error checking for out of bounds motor driver current trip limit
// 4.2.2 ESP32 beta
//       JOIN/UMJOIN on ESP32
// 4.2.1 ESP32 alpha
//       Ready for alpha test on ESP32. Track switching with <=> untested
//       Send DCC signal on MAIN
//       Detects ACK on PROG
// 4.2.0 Track Manager additions:
//       Broadcast improvements to separate <> and Withrottle responses
//       Float eliminated saving >1.5kb PROGMEM and speed. 
//       SET_TRACK(track,mode) Functions (A-H, MAIN|PROG|DC|DCX|OFF)
//       New DC track function and DCX reverse polarity function
//       TrackManager DCC & DC up to 8 Districts Architecture 
//       Automatic ALIAS(name) 
//       Command Parser now accepts Underscore in Alias Names
// 4.1.1 Bugfix: preserve turnout format
//       Bugfix: parse multiple commands in one buffer string correct
//       Bugfix: </> command signal status in Exrail
// 4.1.0 ...
//
// 4.0.2 EXRAIL additions:
//       ACK defaults set to 50mA LIMIT, 2000uS MIN, 20000uS MAX
//       myFilter automatic detection (no need to call setFilter)
//       FIX negative route ids in WIthrottle problem. 
//       IFRED(signal_id), IFAMBER(signal_id), IFGREEN(signal_id)
//       </RED signal_id> </AMBER signal_id> </GREEN signal_id> commands
//       <t cab> command to obtain current throttle settings 
//       JA, JR, JT commands to obtain route, roster and turnout descriptions
//       HIDDEN turnouts
//       PARSE <> commands in EXRAIL
//       VIRTUAL_TURNOUT
//       </KILL ALL> and KILLALL command to stop all tasks. 
//       FORGET forgets the current loco in DCC reminder tables.
//       Servo signals (SERVO_SIGNAL) 
//       High-On signal pins (SIGNALH)
//       Wait for analog value (ATGTE, ATLT)  
// 4.0.1 Small EXRAIL updates
//       EXRAIL BROADCAST("msg") 
//       EXRAIL POWERON
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
