

The DCC-EX Team is pleased to release CommandStation-EX-v3.0.0 as a Production Release.  This release is a major re-write of earlier versions.  We've re-architected the code-base so that it can better handle new features going forward.  Download the compressed files here:

**Downloads (zip and tar.gz) below. These are named without version number in the folder name to make the Arduino IDE happy.**

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/files/5611333/CommandStation-EX.zip)
[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/files/5611335/CommandStation-EX.tar.gz)


**Known Bugs:**
 - **Consisting through JMRI** - currently does not work in this release.  A number of testers were able to develop a work around.  If interested enter a Support Ticket.
 - **Wi-Fi** - works, but can be challenging to use if you want to switch between AP mode and STA station mode.
 - **Pololu Motor Shield** - is supported with this release, but the user may have to play around with some timings to enable programming mode due to limitation in its current sensing circuitry 

 **Summary of the key new features added to CommandStation-EX V3.0.0:**
 - **WiFi Support** - AP and station modes supported. Auto-detection of an ESP8266 WiFi module with AT firmware on a Mega's serial port. Connection to JMRI and WiThrottle clients.
 - **Withrottle Integrations** - Act as a host for four WiThrottle clients concurrently. 
 - **Add LCD/OLED support** - OLED supported on Mega only
 - **Improved CV programming routines** - checks for length of CV pulse, and breaks out of the wait state once it has received an ACK, now reading one CV per second.
 - **Improved current sensing** - rewrote current sensing routines for safer operation. Current thresholds based on milliamps, not magic numbers
 - **Individual track power control** - Ability to toggle power on either or both tracks, and to "JOIN" the tracks and make them output the same waveform for multiple power districts.
 - **Single or Dual-Pin PWM output** - Allows control of H-bridges with PH/EN or dual PWM inputs
 - **New, simpler function command** - ```<F>``` command allows setting functions based on their number, not based on a code as in ```<f>```
 - **Function reminders** - Function reminders are sent in addition to speed reminders
 - **Functions to F28** - All NMRA functions are now supported
 - **Filters and user functions** - Ability to filter commands in the parser and execute custom code based on them
 - **Diagnostic ```<D>``` commands** - See documentation for a full list of new diagnostic commands
 - **Rewrote DCC++ Parser** - more efficient operation, accepts multi-char input and uses less RAM
 - **Rewritten waveform generator** - capable of using any pin for DCC waveform out, eliminating the need for jumpers
 - **Rewritten packet generator** - Simplify and make smaller, remove idea of "registers" from original code
 - **Add free RAM messages** - Free RAM messages are now printed whenever there is a decerase in available RAM
 - **Fix EEPROM bugs**
 - **Number of locos discovery command** - ```<#>``` command 
 - **Support for more locomotives** - 20 locomotives on an UNO and 50 an a Mega.
 - **Automatic slot managment** - slot variable in throttle/function commands are ignored and slot management is taken care of automatically. ```<!>``` command added to release locos from memory.


**Key Contributors**

**Project Lead**
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)

**CommandStation-EX Developers**
- Chris Harlow - Bournemouth, UK (UKBloke)
- Harald Barth - Stockholm, Sweden (Haba)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- M Steve Todd - - Engine Driver and JMRI Interface
- Scott Catalanno - Pennsylvania
- Gregor Baues - Île-de-France, France (grbba)

**exInstaller Software**
- Anthony W - Dayton, Ohio, USA (Dex, Dex++)

**Website and Documentation**
- Mani Kumar - Bangalor, India (Mani / Mani Kumar)
- Fred Decker - Holly Springs, North Carolina, USA (FlightRisk)
- Dave Cutting - Logan, Utah, USA (Dave Cutting/ David Cutting)
- Roger Beschizza - Dorset, UK (Roger Beschizza)
- Keith Ledbetter - Chicago, Illinois, USA (Keith Ledbetter)
-Kevin Smith - (KCSmith)

**Beta Testing / Release Management / Support**
- Larry Dribin	- Release Management
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

[CommandStation-EX.zip](https://github.com/DCC-EX/CommandStation-EX/files/5611333/CommandStation-EX.zip)
[CommandStation-EX.tar.gz](https://github.com/DCC-EX/CommandStation-EX/files/5611335/CommandStation-EX.tar.gz)

