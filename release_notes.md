The DCC-EX Team is pleased to release CommandStation-EX-v3.1.0 as a Production Release.  Release v3.1.0 is a minor release that adds additional features and fixes a number of bugs. With the number of new features, this could have easily been a major release. The team is continually improving the architecture of DCC++EX to make it more flexible and optimizing the code so as to get more performance from the Arduino (and other) microprocessors.  This release includes all of the Point Releases from v3.0.1 to v3.0.16.

**Downloads (zip and tar.gz) below. These are named without version number in the folder name to make the Arduino IDE happy.**

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/releases/download/v3.1.0-Prod/CommandStation-EX.zip)
[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/releases/download/v3.1.0-Prod/CommandStation-EX.tar.gz)

**Known Issues**

- **Wi-Fi** - works, but requires sending <AT> commands from a serial monitor if you want to switch between AP mode and STA station mode after initial setup
- **Pololu Motor Shield** - is supported with this release, but the user may have to adjust timings to enable programming mode due to limitation in its current sensing circuitry

#### Summary of key features and/or bug fixes by Point Release

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

**CommandStation-EX Developers**

- Chris Harlow - Bournemouth, UK (UKBloke)
- Harald Barth - Stockholm, Sweden (Haba)
- Neil McKechnie - Worcestershire, UK (NeilMck)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- M Steve Todd -  
- Scott Catalano - Pennsylvania
- Gregor Baues - Île-de-France, France (grbba)

**Engine Driver and JMRI Interface**

- M Steve Todd

**exInstaller Software**

- Anthony W - Dayton, Ohio, USA (Dex, Dex++)

**Website and Documentation**

- Mani Kumar - Bangalor, India (Mani / Mani Kumar)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- Roger Beschizza - Dorset, UK (Roger Beschizza)
- Keith Ledbetter - Chicago, Illinois, USA (Keith Ledbetter)
- Kevin Smith - Rochester Hills, Michigan USA (KC Smith)

**WebThrotle-EX**

- Fred Decker - Holly Springs, NC (FlightRisk/FrightRisk)
- Mani Kumar - Bangalor, India (Mani /Mani Kumar)
- Matt H - Somewhere in Europe

**Beta Testing / Release Management / Support**

- Larry Dribin - Release Management
- Kevin Smith - Rochester Hills, Michigan USA (KC Smith)
- Keith Ledbetter
- BradVan der Elst
- Andrew Pye
- Mike Bowers
- Randy McKenzie
- Roberto Bravin
- Sim Brigden
- Alan Lautenslager
- Martin Bafver
- Mário André Silva
- Anthony Kochevar
- Gajanatha Kobbekaduwe
- Sumner Patterson
- Paul - Virginia, USA

**Downloads (zip and tar.gz) below. These are named without version number in the folder name to make the Arduino IDE happy.**

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/releases/download/v3.1.0-Prod/CommandStation-EX.zip)
[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/releases/download/v3.1.0-Prod/CommandStation-EX.tar.gz)
