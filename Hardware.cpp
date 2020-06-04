#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include "Hardware.h"
#include "Config.h"
#include "DIAG.h"

#if defined(ARDUINO_ARCH_AVR)
    #include <DIO2.h>
    #define WritePin digitalWrite2
#else
    #define WritePin digitalWrite
#endif
    
void Hardware::init() {
  pinMode(MAIN_POWER_PIN, OUTPUT);
  pinMode(MAIN_SIGNAL_PIN, OUTPUT);
  if (MAIN_SIGNAL_PIN_ALT) pinMode(MAIN_SIGNAL_PIN_ALT, OUTPUT);
  pinMode(MAIN_SENSE_PIN, INPUT);

  pinMode(PROG_POWER_PIN, OUTPUT);
  pinMode(PROG_SIGNAL_PIN, OUTPUT);
  if (PROG_SIGNAL_PIN_ALT) pinMode(PROG_SIGNAL_PIN_ALT, OUTPUT);
  pinMode(PROG_SENSE_PIN, INPUT);
}

void Hardware::setPower(bool isMainTrack, bool on) {
  WritePin(isMainTrack ? MAIN_POWER_PIN : PROG_POWER_PIN, on ? HIGH : LOW);
}
void Hardware::setBrake(bool isMainTrack, bool on) {
  WritePin(isMainTrack ? MAIN_BRAKE_PIN : PROG_BRAKE_PIN, on ? LOW:HIGH);
}

void Hardware::setSignal(bool isMainTrack, bool high) {
  byte pin = isMainTrack ? MAIN_SIGNAL_PIN : PROG_SIGNAL_PIN;
  byte pin2 = isMainTrack ? MAIN_SIGNAL_PIN_ALT : PROG_SIGNAL_PIN_ALT;
  WritePin(pin, high ? HIGH : LOW);
  if (pin2) WritePin(pin2, high ? LOW : HIGH);
}

int Hardware::getCurrentMilliamps(bool isMainTrack) {
  int pin = isMainTrack ? MAIN_SENSE_PIN : PROG_SENSE_PIN;
  float factor = isMainTrack ? MAIN_SENSE_FACTOR : PROG_SENSE_FACTOR;
  int rawCurrent = analogRead(pin);
  return (int)(rawCurrent * factor);
}

void Hardware::setCallback(int duration, void (*isr)()) {
  Timer3.initialize(duration);
  Timer3.disablePwm(TIMER3_A_PIN);
  Timer3.disablePwm(TIMER3_B_PIN);
  Timer3.attachInterrupt(isr);
}

void Hardware::setSingleCallback(int duration, void (*isr)()) {
  Timer1.initialize(duration);
  Timer1.disablePwm(TIMER1_A_PIN);
  Timer1.disablePwm(TIMER1_B_PIN);
  Timer1.attachInterrupt(isr);
}

void Hardware::resetSingleCallback(int duration) {
  if (duration==0) Timer1.stop();
  else Timer1.initialize(duration);
}
