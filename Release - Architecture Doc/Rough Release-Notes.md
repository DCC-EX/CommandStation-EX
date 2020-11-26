# CommandStation-EX Release Notes

## v3.0.0

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
 - **Support for more decoders** - Support for 20 (Uno) or 50 (Mega) mobile decoders, number automaticlaly recognized by JMRI.
 - **Automatic slot managment** - slot variable in throttle/function commands are ignored and slot management is taken care of automatically. ```<!>``` command added to release locos from memory.
