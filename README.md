# CommandStation-EX

Welcome to the future of DCC++ and affordable model railroading. CommandStation-EX represents a complete re-write of the original DCC++ for Arduino. It features:
- WiFi connection to JMRI, or WiThrottle clients *without* JMRI
- Less memory usage than the original DCC++, allowing more slots for locomotives
- Configuration *without* jumpers - all platforms and motor shields.
- Software support for RailCom on the Wasatch Scale Models FireBox boards.
- A cleaner structure for easier feature addition and maintenance
- Support more processors

The DCC waveform generator and main functions have been split off into an API that can be used in your own program for layout automation or just about anything else. The API is used by the JMRI and WiThrottle parts of the code. 

**README is a work in progress**