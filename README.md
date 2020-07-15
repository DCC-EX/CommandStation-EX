# CVReader
Don't be fooled by the name, this chunk of code is an almost complete replacement for DCC++ with the added feature of WiThrottle support WITHOUT a JMRI server.

This repo is acting as a prototyping area for features being built into the DCC-EX CommandStation.

Unlike DCC++ the code here is more cleanly structured and so is easier to maintain and expand. It runs in less memory than DCC++ and uses less interrupts and timers to achieve a more reliable DCC signal. You will not find a PacketRegister file with a bizarre mix of string parsing, packet building, queue managing, timer fiddling and hardware pins all mashed into one over-complex class. 



It has an API interface to the DCC commands that can be used by your own bespoke sketch.
This API is actually used by the JMRI command parser and the WiThrottle command parser. 
The CVReader.ino sketch provides an example of setting this up with Wifi and JMRI command parsing.

In addition, program track operations requiring ACK responses do not cause blocking of the Arduino loop() and so will not hold up throttle requests.
Loco Functions F0-F12 will issue reminders automatically.
There is a much higher register limit (ie more locos) in the same memory. 
You can also create your own commands or filter out commands sent by JMRI to suit your own layout.
  


