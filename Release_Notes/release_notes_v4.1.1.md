Version 4.1.1 Release Notes
*************************

The DCC-EX Team is pleased to release CommandStation-EX v4.1.1 as a Production Release for the general public.
This release is a Minor release with many significant EX-RAIL enhancements and new automation features in addition to some bug fixes. 
The team continues improving the architecture of DCC++EX to make it more flexible and optimizing the code to get more performance from the Arduino (and other) microprocessors.  This release includes all of the Point Releases from v4.0.1 to v4.1.1 rc13.

**Downloads (zip and tar.gz) below. These are named without version number in the folder name to make the Arduino IDE happy.**

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/releases/download/v4.1.1-Prod/CommandStation-EX.zip)



[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/releases/download/v4.1.1-Prod/CommandStation-EX.tar.gz)

**New Command Station & EX-RAIL Features**
  - ACK defaults are now set to LIMIT 50mA, MIN 2000uS, MAX 20000uS for more compatibility with non NMRA compliant decoders
  - Automatically detect and run a myFilter add-on (no need to call setFilter)

  -  New Commands for the Arduino IDE Serial Monitor and JMRI DCC++ Traffic Monitor
     -  </RED signal_id>   to turn a individual LED Signal On & Off
     -  </AMBER signal_id> "
     -  </GREEN signal_id> "
     -  </KILL ALL> command to stop all tasks, and Diagnostic messages when KILL is used
     -  < t cab> command to obtain current throttle setting

   -  Allow WRITE CV on PROG <W CV VALUE>
   -  Updated CV read command <R cv>. Equivalent to <V cv 0>.  Uses the verify callback.
   -  Allow WRITE CV on PROG <W CV VALUE)
   -  Change callback parameters are now optional on PROG
  
  - New JA, JR, JT commands availabe for Throttle Developers to obtain Route, Roster and Turnout descriptions for communications

  - New EX-RAIL Functions to use in Automation(n), ROUTE(N) & SEQUENCE(N) Scripts
    - ATGTE & ATLT wait for analog value, (At Greater Than or Equal and At Less Than a certain value)
    - FADE command now works for LEDs connected on PCA9685 Servo/Signal board Output vpins 
    - FORGET Forgets the current loco in DCC reminder tables saving memory and wasted packets sent to the track
    - "IF" signal detection with IFRED(signal_id), IFAMBER(signal_id), IFGREEN(signal_id) 
    - KILLALL command to stop all tasks, and Diagnostic messages when KILL is used
    - PARSE <> commands in EXRAIL allows sending of DCC-EX commands from EX-RAIL
    - SERVO_SIGNAL Servo signals assigned to a specific servo turnout 
    - SIGNALH High-On signal pins (Arduino normally handles active LOW signals. This allows for active HIGH)
    - HIDDEN turnouts (hide a REAL turnout and create a VIRTUAL turnout to handle actions that happen BEFORE a turnout is thrown)
    - VIRTUAL_TURNOUT definition

**EX-RAIL Updates**
  - EXRAIL BROADCAST("msg") sends any message to all throttles/JMRI via serial and WiFi
  - EXRAIL POWERON turns on power to both tracks from EX-RAIL (the equivalent of sending the <1> command)

** Other Enhancements**
  - UNO Progmem is optimize to allow for small EXRAIL Automation scipts to run within the limited space for testing purposes.
  - PCA9685 Servo Signal board supports 'Nopoweroffleds', servo pins stay powered on after position reached, otherwise the new FADE would always turn off.
  - Position servo can use spare servo pin as a GPIO pin. 

**4.1.1 Bug Fixes**

- Preserve the turnout format
- Parse multiple commands in one buffer string currectly
- Fix </> command signal status in EX-RAIL
- Read long loco addresses in EX-RAIL
- FIX negative route IDs in WIthrottle 

See the version.h file for notes about which of the 4.1.1 features were added/changed by point release.

**Known Issues**

- **Wi-Fi** - Requires sending `<AT>` commands from a serial monitor if you want to switch between AP mode and STA station mode after initial setup
- **Pololu Motor Shield** - is supported with this release, but the user may have to adjust timings to enable programming mode due to limitations in its current sensing circuitry

**All New Major DCC++EX 4.0.0 features**

- **New HAL Hardware Abstraction Layer API** that automatically detects and greatly simplifies interfacing to many predefined accessory boards for servos, signals & sensors and added I/O (digital and analog inputs and outputs, servos etc). 
- HAL Support for;
  - MCP23008, MCP23017 and PCF9584 I2C GPIO Extender modules. 
  - PCA9685 PWM (servo & signal) control modules. 
  - Analogue inputs on Arduino pins and on ADS111x I2C modules. 
  - MP3 sound playback via DFPlayer module. 
  - HC-SR04 Ultrasonic range sensor module. 
  - VL53L0X Laser range sensor module (Time-Of-Flight). 
  - A new `<D HAL SHOW>` command to list the HAL devices attached to the command station
 
**New Command Station Broadcast throttle logic**

- Synchronizes multiple WiThrottles and PC based JMRI Throttles for direction, speed and F-key updates

**New ‘Discovered Servers’ on WiFi Throttles**

- Our New multicast Dynamic Network Server (mDNS) enhancement allows us to display the available WiFi server connections to a DCC++EX Command Station. Selecting it allows your WiThrottle App to connect to and load Server Rosters and function keys to your throttle from the new DCC++EX Command Station Server Roster.

**New DCC++EX 4.0.0 with EX-RAIL Extended Railroad Automation Instruction Language**

- Use to control your entire layout or as a separate accessory/animation controller
- Awesome, cleverly powerful yet simple user friendly scripting language for user built Automation & Routing scripts. 
- You can control Engines, Sensors, Turnouts, Signals, Outputs and Accessories that are entered into your new myAutomation.h file, then uploaded into the DCC++EX Command Station.
- EX-RAIL scripts are automatically displayed as Automation {Handoff} and Route {Set} buttons on supported WiFi Throttle Apps.

**New EX-RAIL ‘Roster’ Feature**

- List and store user defined engine roster & function keys inside the command station, and automatically load them in WiFi Throttle Apps.
- When choosing “DCC++EX” from discovered servers an Engine Driver or WiThrottle is directly connected to the Command Station. 
- The EX-RAIL ’ROSTER’ command allows all the engine numbers, names and function keys you’ve listed in your myAutomation.h file to automatically upload the Command Station's ‘Server Roster’ into your Engine Driver and WiThrottle Apps.  

**New JMRI 4.99.2 and above specific DCC++EX 4.0 features**

- Enhanced JMRI DCC++ Configure Base Station pane for building and maintaining Sensor, Turnout and Output devices, or these can automatically be populated from the DCC++EX Command Station's mySetup.h file into JMRI.

- JMRI now supports multiple serial connected DCC++EX Command Stations, to display and track separate "Send DCC++ Command" and "DCC++ Traffic" Monitors for each Command Station at the same time.
  For example: Use an Uno DCC++EX DecoderPro Programming Station {DCC++Prg} on a desktop programming track and a second Mega DCC++EX EX-RAIL Command Station for Operations {DCC++Ops} on the layout with an additional `<JOINED>` programming spur or siding track for acquiring an engine and ‘Drive Away’ onto the mainline (see the DriveAway feature for more information).

**DCC++EX 4.0.0 additional product enhancements**

- Additional Motor Shields and Motor Board {boosters) supported
- Additional Accessory boards supported for GPIO expansion, Sensors, Servos & Signals
- Additional diagnostic commands like ‘D ACK RETRY’ and ‘D EXRAIL ON’ events, ‘D HAL SHOW’ devices and ‘D SERVO’ positions, and ‘D RESET’ the command station while maintaining the serial connection with JMRI
- Automatic retry on failed ACK detection to give decoders another chance
- New EX-RAIL ’/’ slash command allows JMRI to directly communicate with many EX-RAIL scripts
- Turnout class revised to expand turnout capabilities and allow turnout names/descriptors to display in WiThrottle Apps.
- Build turnouts through either or both mySetup.h and myAutomation.h files, and have them automatically passed to, and populate, JMRI Turnout Tables
- Turnout user names display in Engine Driver & WiThrottles
- Output class now allows ID > 255. 
- Configuration options to globally flip polarity of DCC Accessory states when driven from `<a>` command and `<T>` command.
- Increased use of display for showing loco decoder programming information. 
- Can disable EEPROM memory code to allow room for DCC++EX 4.0 to fit on a Uno Command Station
- Can define border between long and short addresses 
- Native non-blocking I2C drivers for AVR and Nano architectures (fallback to blocking Wire library for other platforms). 
- EEPROM layout change - deletes EEPROM contents on first start following upgrade. 

**4.0.0 Bug Fixes**

- Compiles on Nano Every
- Diagnostic display of ack pulses >32ms
- Current read from wrong ADC during interrupt
- AT(+) Command Pass Through 
- CiDAP WiFi Drop out and the WiThrottle F-key looping error corrected
- One-off error in CIPSEND drop
- Common Fault Pin Error
- Uno Memory Utilization optimized

#### Summary of Release 3.1.0 key features and/or bug fixes by Point Release

**Summary of the key new features added to CommandStation-EX V3.0.16**

- Ignore CV1 bit 7 read if rejected by a non NMRA compliant decoder when identifying loco id

**Summary of the key new features added to CommandStation-EX V3.0.15**

- Send function commands just once instead of repeating them 4 times

**Summary of the key new features added to CommandStation-EX V3.0.14**

- Add feature to tolerate decoders that incorrectly have gaps in their ACK pulse
- Provide proper track power management when joining and unjoining tracks with <1 JOIN>

**Summary of the key new features added to CommandStation-EX V3.0.13**

- Fix for CAB Functions greater than 127

**Summary of the key new features added to CommandStation-EX V3.0.12**

- Fixed clear screen issue for nanoEvery and nanoWifi

**Summary of the key new features added to CommandStation-EX V3.0.11**

- Reorganized files for support of 128 speed steps

**Summary of the key new features added to CommandStation-EX V3.0.10**

- Added Support for the Teensy 3.2, 3.5, 3.6, 4.0 and 4.1 MCUs
- No functional change just changes to avoid complier warnings for Teensy/nanoEvery

**Summary of the key new features added to CommandStation-EX V3.0.9**

- Rearranges serial newlines for the benefit of JMRI
- Major update for efficiencies in displays (LCD, OLED)
- Add I2C Support functions

**Summary of the key new features added to CommandStation-EX V3.0.8**

- Wraps <* *> around DIAGS for the benefit of JMRI

**Summary of the key new features added to CommandStation-EX V3.0.7**

- Implemented support for older 28 apeed step decoders - Option to turn on 28 step speed decoders in addition to 128. If set, all locos will use 28 steps.
- Improved overload messages with raw values (relative to offset)

**Summary of the key new features added to CommandStation-EX V3.0.6**

- Prevent compiler warning about deprecated B constants
- Fix Bug that did not let us transmit 5 byte sized packets - 5 Byte commands like PoM (programming on main) were not being sent correctly
- Support for Huge function numbers (DCC BinaryStateControl) - Support Functions beyond F28
- <!> ESTOP all - New command to emergency stop all locos on the main track
- <- [cab]> estop and forget cab/all cabs - Stop and remove loco from the CS. Stops the repeating throttle messages
- `<D RESET>` command to reboot Arduino
- Automatic sensor offset detect
- Improved startup msgs from Motor Drivers (accuracy and auto sense factors)
- Drop post-write verify - No need to double check CV writes. Writes are now even faster.
- Allow current sense pin set to UNUSED_PIN - No need to ground an unused analog current pin. Produce startup warning and callback -2 for prog track cmds.

**Summary of the key new features added to CommandStation-EX V3.0.5**

- Fix Fn Key startup with loco ID and fix state change for F16-28
- Removed ethernet mac config and made it automatic
- Show wifi ip and port on lcd
- Auto load config.example.h with warning
- Dropped example .ino files
- Corrected .ino comments
- Add Pololu fault pin handling
- Waveform speed/simplicity improvements
- Improved pin speed in waveform
- Portability to nanoEvery and UnoWifiRev2 CPUs
- Analog read speed improvements
- Drop need for DIO2 library
- Improved current check code
- Linear command
- Removed need for ArduinoTimers files
- Removed option to choose different timer
- Added EX-RAIL hooks for automation in future version
- Fixed Turnout list
- Allow command keywords in mixed case
- Dropped unused memstream
- PWM pin accuracy if requirements met

**Summary of the key new features added to CommandStation-EX V3.0.4**

- "Drive-Away" Feature - added so that throttles like Engine Driver can allow a loco to be programmed on a usable, electrically isolated programming track and then drive off onto the main track
- WiFi Startup Fixes

**Summary of the key new features added to CommandStation-EX V3.0.3**

- Command to write loco address and clear consist
- Command will allow for consist address
- Startup commands implemented

**Summary of the key new features added to CommandStation-EX V3.0.2:**

- Create new output for current in mA for `<c>` command - New current response outputs current in mA, overlimit current, and maximum board capable current
- Simultaneously update JMRI to handle new current meter

**Summary of the key new features added to CommandStation-EX V3.0.1:**

- Add back fix for jitter
- Add Turnouts, Outputs and Sensors to `<s>` command output

**CommandStation-EX V3.0.0:**

**Release v3.0.0 was a major rewrite if earlier versions of DCC++.  The code base was re-architeced and core changes were made to the Waveform generator to reduce overhead and make better use of Arduino.** **Summary of the key new features added in Release v3.0.0 include:**

- **New USB Browser Based Throttle** - WebThrottle-EX is a full front-end to controller to control the CS to run trains.
- **WiFi Support** - AP and station modes supported. Auto-detection of an ESP8266 WiFi module with AT firmware on a Mega's serial port. Connection to JMRI and WiThrottle clients.
- **Withrottle Integrations** - Act as a host for up to four WiThrottle clients concurrently.
- **Add LCD/OLED support** - OLED supported on Mega only
- **Improved CV programming routines** - checks for length of CV pulse, and breaks out of the wait state once it has received an ACK, now reading one CV per second.
- **Improved current sensing** - rewrote current sensing routines for safer operation. Current thresholds based on milliamps, not magic numbers
- **Individual track power control** - Ability to toggle power on either or both tracks, and to "JOIN" the tracks and make them output the same waveform for multiple power districts.
- **Single or Dual-Pin PWM output** - Allows control of H-bridges with PH/EN or dual PWM inputs
- **New, simpler function command** - `<F>` command allows setting functions based on their number, not based on a code as in `<f>`
- **Function reminders** - Function reminders are sent in addition to speed reminders
- **Functions to F28** - All NMRA functions are now supported
- **Filters and user functions** - Ability to filter commands in the parser and execute custom code based on them. (ex: Redirect Turnout commands via NRF24)
- **Diagnostic `<D>` commands** - See documentation for a full list of new diagnostic commands
- **Rewrote DCC++ Parser** - more efficient operation, accepts multi-char input and uses less RAM
- **Rewritten waveform generator** - capable of using any pin for DCC waveform out, eliminating the need for jumpers
- **Rewritten packet generator** - Simplify and make smaller, remove idea of "registers" from original code
- **Add free RAM messages** - Free RAM messages are now printed whenever there is a decerase in available RAM
- **Fix EEPROM bugs**
- **Number of locos discovery command** - `<#>` command
- **Support for more locomotives** - 20 locomotives on an UNO and 50 an a Mega.
- **Automatic slot management** - slot variable in throttle/function commands are ignored and slot management is taken care of automatically. `<->` and `<- CAB>` commands added to release locos from memory and stop packets to the track.

**Key Contributors**

**Project Lead**

- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)

**EX-CommandStation Developers**

- Chris Harlow - Bournemouth, UK (UKBloke)
- Harald Barth - Stockholm, Sweden (Haba)
- Neil McKechnie - Worcestershire, UK (NeilMck)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- M Steve Todd - Oregon, USA (MSteveTodd) 
- Scott Catalano - Pennsylvania
- Gregor Baues - Île-de-France, France (grbba)

**Engine Driver and JMRI Interface**

- M Steve Todd

**EX-Installer Software**

- Anthony W - Dayton, Ohio, USA (Dex, Dex++)

**Website and Documentation**

- Mani Kumar - Bangalor, India (Mani / Mani Kumar)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- Roger Beschizza - Dorset, UK (Roger Beschizza)
- Keith Ledbetter - Chicago, Illinois, USA (Keith Ledbetter)
- Kevin Smith - Rochester Hills, Michigan USA (KC Smith)
- Colin Grabham - Central NSW, Australia (Kebbin)
- Peter Cole - Brisbane, QLD, Australia (peteGSX)
- Peter Akers - Brisbane, QLD, Australia (flash62au)

**EX-WebThrottle**

- Fred Decker - Holly Springs, NC (FlightRisk/FrightRisk)
- Mani Kumar - Bangalor, India (Mani /Mani Kumar)
- Matt H - Somewhere in Europe
  
**Hardware / Electronics**

- Harald Barth - Stockholm, Sweden (Haba)
- Paul Antoine, Western Australia
- Neil McKechnie - Worcestershire, UK
- Fred Decker - Holly Springs NC, USA
- Herb Morton - Kingwood Texas, USA (Ash++)

**Beta Testing / Release Management / Support**

- Larry Dribin - Release Management
- Kevin Smith - Rochester Hills, Michigan USA (KC Smith)
- Herb Morton - Kingwood Texas, USA (Ash++)
- Keith Ledbetter
- Brad Van der Elst
- Andrew Pye
- Mike Bowers
- Randy McKenzie
- Roberto Bravin
- Sam Brigden
- Alan Lautenslager
- Martin Bafver
- Mário André Silva
- Anthony Kochevar
- Gajanatha Kobbekaduwe
- Sumner Patterson
- Paul - Virginia, USA

**Downloads (zip and tar.gz) below. These are named without version number in the folder name to make the Arduino IDE happy.**

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/releases/download/v4.1.1-Prod/CommandStation-EX.zip)


[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/releases/download/v4.1.1-Prod/CommandStation-EX.tar.gz)
