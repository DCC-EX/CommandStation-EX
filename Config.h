#ifndef Config_h
#define Config_h

// This hardware configuration would normally be setup using a bunch of #ifdefs.

const byte MAIN_POWER_PIN = 3;
const byte MAIN_SIGNAL_PIN = 12;
const byte MAIN_SIGNAL_PIN_ALT = 0;  // for hardware that flipflops signal pins 
const byte MAIN_SENSE_PIN = A0;   
const byte MAIN_SENSE_FACTOR=1; //  analgRead(MAIN_SENSE_PIN) * MAIN_SENSE_FACTOR = milliamps 
const byte MAIN_BRAKE_PIN = 9;

const byte PROG_POWER_PIN = 11;
const byte PROG_SIGNAL_PIN = 13;
const byte PROG_SIGNAL_PIN_ALT = 0;  // for hardware that flipflops signal pins 
const byte PROG_SENSE_PIN = A1;
const byte PROG_BRAKE_PIN = 10;

const float PROG_SENSE_FACTOR=1; //  analgRead(PROG_SENSE_PIN) * PROG_SENSE_FACTOR = milliamps 

// Allocations with memory implications.. however tiny!
// Base system takes approx 500 bytes + 4 per loco and 3 per turnout.
const byte MAX_LOCOS=50;             
const byte MAX_TURNOUTS=50;

#endif
