// This hardware configuration would normally be setup using a bunch of #ifdefs.

const byte MAIN_POWER_PIN = 3;
const byte MAIN_SIGNAL_PIN = 12;
const byte MAIN_SIGNAL_PIN_ALT = 0;  // for hardware that flipflops signal pins 
const byte MAIN_SENSE_PIN = A0;   
const byte MAIN_SENSE_FACTOR=1; //  analgRead(MAIN_SENSE_PIN) * MAIN_SENSE_FACTOR = milliamps 

const byte PROG_POWER_PIN = 11;
const byte PROG_SIGNAL_PIN = 13;
const byte PROG_SIGNAL_PIN_ALT = 0;  // for hardware that flipflops signal pins 
const byte PROG_SENSE_PIN = A1;
const float PROG_SENSE_FACTOR=1; //  analgRead(PROG_SENSE_PIN) * PROG_SENSE_FACTOR = milliamps 
