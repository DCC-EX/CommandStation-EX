#include <Arduino.h>
#include <TimerThree.h>

#include "Hardware.h"
#include "Config.h"

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
  digitalWrite(isMainTrack ? MAIN_POWER_PIN : PROG_POWER_PIN, on ? HIGH : LOW);
}

void Hardware::setSignal(bool isMainTrack, bool high) {
  byte pin = isMainTrack ? MAIN_SIGNAL_PIN : PROG_SIGNAL_PIN;
  byte pin2 = isMainTrack ? MAIN_SIGNAL_PIN_ALT : PROG_SIGNAL_PIN_ALT;
  digitalWrite(pin, high ? HIGH : LOW);
  if (pin2) digitalWrite(pin2, high ? LOW : HIGH);
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
