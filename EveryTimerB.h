// EveryTimerB library.
// by Kees van der Oord Kees.van.der.Oord@inter.nl.net

// Timer library for the TCB timer of the AtMega4809 processor.
// tested on the Arduino Nano Every (AtMega4809) and the Arduino 1.8.12 IDE
// support for the Every is the 'Arduino MegaAVR' boards module (Tools | Board | Boards Manager)

// usage:
/*
#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"
#define Timer1 TimerB2    // use TimerB2 as a drop in replacement for Timer1
#else // assume architecture supported by TimerOne library ....
#include "TimerOne.h"
#endif

// code below will now work both on the MegaAVR and AVR processors

void setup() {
  Timer1.initialize();
  Timer1.attachInterrupt(myisr);
  Timer1.setPeriod(1000000UL);     // like the TimerOne library this will start the timer as well
}

void myisr() {
  // do something useful every second  
}
*/
// clock source options:
// The TCB clock source is specified in the initialize() function with default value EveryTimerB_CLOCMODE.
// define this macro before including this file to use a different default clock mode
// e.g.:
// #define EveryTimerB_CLOCMODE TCB_CLKSEL_CLKTCA_gc  // 250 kHz ~ 4 us
// #define EveryTimerB_CLOCMODE TCB_CLKSEL_CLKDIV2_gc //   8 MHz ~ 0.125 us
// #define EveryTimerB_CLOCMODE TCB_CLKSEL_CLKDIV_gc  //  16 MHz ~ 0.0625 us  

// timer options
// The 4809 has one A timer (TCA) and four B timers (TCB).
// TCA and TCB3 are used by the arduino core to generate the clock used by millis() and micros().
// TCB0 generates the PWM timing for pin D6, TCB1 for pin D3.
// By default Timer Control B2 is defined as TimerB2 in the EveryTimerB library.
// If you would like to use the TCB0 and TCB1 as well you have to copy the code
// from the EveryTimerB.cpp into your product file and adapt for B0 and B1 timers.
//
// for information on the 4809 TCA and TCB timers:
// http://ww1.microchip.com/downloads/en/AppNotes/TB3217-Getting-Started-with-TCA-90003217A.pdf
// http://ww1.microchip.com/downloads/en/Appnotes/TB3214-Getting-Started-with-TCB-90003214A.pdf
// %LOCALAPPDATA%\Arduino15\packages\arduino\hardware\megaavr\1.8.5\cores\arduino\wiring.c
// %LOCALAPPDATA%\Arduino15\packages\arduino\hardware\megaavr\1.8.5\variants\nona4809\variant.c
// %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino5\avr\include\avr\iom4809.h

// 20 MHz system clock
// to run the Every at 20 MHz, add the lines below to the nona4809 section of the boards.txt file
// in %LOCALAPPDATA%\Arduino15\packages\arduino\hardware\megaavr\1.8.5.
// they add the sub menu 'Tools | Clock' to choose between 16MHz and 20MHz.
/*
menu.clock=Clock
nona4809.menu.clock.16internal=16MHz
nona4809.menu.clock.16internal.build.f_cpu=16000000L
nona4809.menu.clock.16internal.bootloader.OSCCFG=0x01
nona4809.menu.clock.20internal=20MHz
nona4809.menu.clock.20internal.build.f_cpu=20000000L
nona4809.menu.clock.20internal.bootloader.OSCCFG=0x02
*/
// On 20Mhz, the 1.8.12 IDE MegaAvr core library implementation
// of the millis() and micros() functions is not accurate.
// the file "MegaAvr20MHz.h" implements a quick hack to correct for this 
//
// to do:
// there is no range check on the 'period' arguments of setPeriod ...
// check if it is necessary to set the CNT register to 0 in start()

#ifndef EveryTimerB_h_
#define EveryTimerB_h_
#ifdef ARDUINO_ARCH_MEGAAVR

#ifndef EveryTimerB_CLOCMODE 
#define EveryTimerB_CLOCMODE TCB_CLKSEL_CLKTCA_gc
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "MegaAvr20MHz.h"
#include "pins_arduino.h"

#define TCB_RESOLUTION 65536UL // TCB is 16 bit
// CLOCK   F_CPU  DIV  TICK      OVERFLOW  OVERFLOW/s
// CLKTCA  16MHz  64   4000  ns  262144us    3.8 Hz
// CLKDIV2 16MHz   2    125  ns    8192us  122 Hz
// CLKDIV1 16MHz   1     62.5ns    4096us  244 Hz
// CLKTCA  20MHz  64   3200  ns  209716us    4.8 Hz
// CLKDIV2 20MHz   2    100  ns    6554us  153 Hz
// CLKDIV1 20MHz   1     50  ns    3277us  305 Hz

class EveryTimerB
{
  public:
    // The AtMega Timer Control B clock sources selection:
    // TCB_CLKSEL_CLKTCA_gc,  // Timer Controller A, Arduino framework sets TCA to F_CPU/64 = 250kHz (4us) @ 16MHz or 312.5kHz (3.2us) @ 20MHz
    // TCB_CLKSEL_CLKDIV2_gc, // CLK_PER/2 Peripheral Clock / 2: 8MHz @ 16Mhz or 10MHz @ 20MHz 
    // TCB_CLKSEL_CLKDIV1_gc  // CLK_PER Peripheral Clock: 16MHz @ 16Mhz or 20MHz @ 20MHz 

    // intialize: sets the timer compare mode and the clock source
    void initialize(TCB_t * timer_ = &TCB2, TCB_CLKSEL_t clockSource = EveryTimerB_CLOCMODE, unsigned long period = 1000000UL) __attribute__((always_inline)) {
      timer = timer_;
#if defined(MegaAvr20MHzCorrected)
      corrected20MHzInit(); // see commment in MegaAvr20MHz_h
#endif        
      stop();
      timer->CTRLB = TCB_CNTMODE_INT_gc & ~TCB_CCMPEN_bm; // timer compare mode with output disabled
      if(clockSource) setClockSource(clockSource);
      if(period) setPeriod(period);
    }
    
    void setClockSource(TCB_CLKSEL_t clockSource) __attribute__((always_inline)) {
      timer->CTRLA = clockSource; // this stops the clock as well ...
      switch(clockSource) {
#if F_CPU == 20000000UL
        case TCB_CLKSEL_CLKTCA_gc:  maxTimeWithoutOverflow = 209715; break;  // (TCB_RESOLUTION * 64) / 20
        case TCB_CLKSEL_CLKDIV2_gc: maxTimeWithoutOverflow =   6553; break;  // (TCB_RESOLUTION *  2) / 20
        case TCB_CLKSEL_CLKDIV1_gc: maxTimeWithoutOverflow =   3276; break;  // (TCB_RESOLUTION *  1) / 20
#else
        case TCB_CLKSEL_CLKTCA_gc:  maxTimeWithoutOverflow = 262144; break;
        case TCB_CLKSEL_CLKDIV2_gc: maxTimeWithoutOverflow =   8192; break;
        case TCB_CLKSEL_CLKDIV1_gc: maxTimeWithoutOverflow =   4096; break;
#endif        
      }
    }

    TCB_CLKSEL_t getClockSource() {
      return (TCB_CLKSEL_t)(timer->CTRLA & (TCB_CLKSEL_CLKTCA_gc|TCB_CLKSEL_CLKDIV2_gc|TCB_CLKSEL_CLKDIV1_gc));
    }

    double getFrequencyOfClock(TCB_CLKSEL_t clock) {
      switch(clock) {
        // suppose nobody touched the default TCA configuration ...
        case TCB_CLKSEL_CLKTCA_gc:  return double(F_CPU/64); break;
        case TCB_CLKSEL_CLKDIV2_gc: return double(F_CPU/2); break;
        case TCB_CLKSEL_CLKDIV1_gc: return double(F_CPU); break;
      }
      return 0.0;
    }

    double getClockFrequency() {
      return getFrequencyOfClock(getClockSource());
    }

    // setPeriod: sets the period
    // note: max and min values are different for each clock 
    // CLKTCA: conversion from us to ticks multiplies 'period' first with 10, so max value is MAX_ULONG/10 ~ 1 hr 11 minutes 34 seconds
    // CLKDIV2: conversion from us to ticks is a *10 multiplication, so max value is 420M us (~ 7 minutes)
    // CLKDIV1: conversion from us to ticks is a *20 multiplication, so max value is 210M us (~ 3.5 minutes)
    void setPeriod(unsigned long period /* us */) __attribute__((always_inline)) {
      timer->CTRLA &= ~TCB_ENABLE_bm;
      // conversion from us to ticks depends on the clock
      switch(timer->CTRLA & TCB_CLKSEL_gm)
      {
        case TCB_CLKSEL_CLKTCA_gc:
#if F_CPU == 20000000UL
          period = (period * 10) / 32; // 20Mhz / 64x clock divider of TCA => 3.2 us / tick
#else // 16000000UL
          period /= 4; // 16MHz / 64x clock divider of TCA => 4 us / tock
#endif        
          break;
        case TCB_CLKSEL_CLKDIV2_gc:
#if F_CPU == 20000000UL
          period *= 10; // 20MHz / 2x clock divider => 10 ticks / us
#else // 16000000UL
          period *= 8;  // 16MHz / 2x clock divider => 8 ticks / us
#endif        
          break;
        case TCB_CLKSEL_CLKDIV1_gc:
#if F_CPU == 20000000UL
          period *= 20; // 20MHz: 20 ticks / us
#else // 16000000UL
          period *= 16; // 16MHz: 16 ticks / u3
#endif
          break;
      }

      // to support longer than TCB_RESOLUTION ticks, 
      // this class supports first waiting for N 'overflowCounts'
      // and next program the timer the remaining 'remainder' ticks:
      countsPerOverflow = TCB_RESOLUTION;
      overflowCounts = period / TCB_RESOLUTION;
      remainder = period % TCB_RESOLUTION;

      // the timer period is always one tick longer than programmed,
      // so a remainder of 1 is not possible. reduce the length of 
      // the 'overflow' cycles to get a remainder that is not 1
      if(overflowCounts) {
        while(remainder == 1) {
          --countsPerOverflow;
          overflowCounts = period / countsPerOverflow;
          remainder = period % countsPerOverflow;
        }
      }

      // the timer period is always one tick longer than programmed
      --countsPerOverflow;
      if(remainder) --remainder;

      // let's go
      start();
    }
    
    void start() __attribute__((always_inline)) {
      stop();
      overflowCounter = overflowCounts;
      timer->CCMP = overflowCounts ? countsPerOverflow : remainder;
      timer->CNT = 0;
      timer->CTRLA |= TCB_ENABLE_bm;
    }
    
    void stop() __attribute__((always_inline)) {
      timer->CTRLA &= ~TCB_ENABLE_bm;
      timer->INTFLAGS = TCB_CAPT_bm;  // writing to the INTFLAGS register will clear the interrupt request flag
    }
    
    bool isEnabled(void) __attribute__((always_inline)) {
      return timer->CTRLA & TCB_ENABLE_bm ? true : false;
    }

    void enable(void) __attribute__((always_inline)) {
      timer->CTRLA |= TCB_ENABLE_bm;
    }

    bool disable(void) __attribute__((always_inline)) {
      timer->CTRLA &= ~TCB_ENABLE_bm;
    }

    void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
      isrCallback = isr;
      timer->INTFLAGS = TCB_CAPT_bm; // clear interrupt request flag
      timer->INTCTRL = TCB_CAPT_bm;  // Enable the interrupt
    }
    
    void attachInterrupt(void (*isr)(), unsigned long microseconds) __attribute__((always_inline)) {
      if(microseconds > 0) stop();
      attachInterrupt(isr);
      if (microseconds > 0) setPeriod(microseconds);
    }
    
    void detachInterrupt() __attribute__((always_inline)) {
      timer->INTCTRL &= ~TCB_CAPT_bm; // Disable the interrupt
      isrCallback = isrDefaultUnused;
    }

    void enableInterrupt() __attribute__((always_inline)) {
      timer->INTFLAGS = TCB_CAPT_bm; // clear interrupt request flag
      timer->INTCTRL = TCB_CAPT_bm;  // Enable the interrupt
    }

    void disableInterrupt() __attribute__((always_inline)) {
      timer->INTCTRL &= ~TCB_CAPT_bm;  // Enable the interrupt
    }

    TCB_CNTMODE_enum getMode() __attribute__((always_inline)) {
      return (TCB_CNTMODE_enum) (timer->CTRLB & 0x7);
    }

    void setMode(TCB_CNTMODE_enum mode) __attribute__((always_inline)) {
      timer->CTRLB = (timer->CTRLB & ~0x7) | mode;
    }

    uint8_t isOutputEnabled() __attribute__((always_inline)) {
      return timer->CTRLB & TCB_CCMPEN_bm;
    }

    uint8_t enableOutput() __attribute__((always_inline)) {
      timer->CTRLB |= TCB_CCMPEN_bm;
    }

    uint8_t disableOutput() __attribute__((always_inline)) {
      timer->CTRLB &= ~TCB_CCMPEN_bm;
    }

    // this will start PWM on pin 6 (TCB0) or pin 3 (TCB1)
    // set the pins to output with setMode(x,OUTPUT) before calling this function
    // period determines the clock ticks in one cycle:
    //  16MHz clock: slowest frequency at 255 = 62 kHz.
    //   8MHz clock: slowest frequency at 255 = 31 kHz.
    // 256kHz clock: slowest frequency at 255 =  1 kHz.
    // compare determines the duty cycle.
    // with a period of 255, set the compare to 128 to get 50% duty cycle.
    void setPwmMode(byte period, byte compare) {
    	disableInterrupt();
    	setMode(TCB_CNTMODE_PWM8_gc);
    	timer->CCMPL = period;
    	timer->CCMPH = compare;
    	enableOutput();
    	enable();
    }

    void getPwmMode(byte & period, byte & compare) {
      period = timer->CCMPL;
      compare = timer->CCMPH;
    }

    void setPwm(double frequency, double dutyCycle) {
      TCB_CLKSEL_t clockSource = TCB_CLKSEL_CLKDIV1_gc;
      double clockFrequency = getFrequencyOfClock(clockSource);
      if(frequency < (clockFrequency/256.)) {
        clockSource = TCB_CLKSEL_CLKDIV2_gc;
        clockFrequency = getFrequencyOfClock(clockSource);
      }
      if(frequency < (clockFrequency/256.)) {
        clockSource = TCB_CLKSEL_CLKTCA_gc;
        clockFrequency = getFrequencyOfClock(clockSource);
      }
      double period = (clockFrequency / frequency) - 1.0 + 0.5;
      if(period > 255.) period = 255.;
      if(period < 0.) period = 0.0;
      double compare = period * dutyCycle + 0.5;
      if(compare < 0.0) compare = 0.0;
      if(compare > period) compare = period;
      setPwmMode((byte)(period),(byte)(compare));
    }

    void getPwm(double & frequency, double & dutyCycle) {
      byte period, compare;
      getPwmMode(period,compare);
      frequency = getClockFrequency() / (((double)period) + 1);
      dutyCycle = (double) compare / (((double)period) + 1);
    }

    void setTimerMode() {
      disable();
      disableOutput();
      setMode(TCB_CNTMODE_INT_gc);
      if(isrCallback != isrDefaultUnused) {
        enableInterrupt();
      }
    }

    TCB_t * getTimer() { return timer; }
    long getOverflowCounts() { return overflowCounts; }
    long getRemainder() { return remainder; }
    long getOverflowCounter() { return overflowCounter; }
    long getOverflowTime() { return maxTimeWithoutOverflow; }
  
//protected:
    // the next_tick function is called by the interrupt service routine TCB0_INT_vect
    //friend extern "C" void TCB0_INT_vect(void);
    void next_tick() __attribute__((always_inline)) {
      --overflowCounter;
      if(overflowCounter > 0) {
        return;
      }
      if(overflowCounter < 0) {
        // finished waiting for remainder
        if (overflowCounts) {
          // restart with a max counter
          overflowCounter = overflowCounts;
          timer->CCMP = countsPerOverflow;
        }
      } else {
        // overflowCounter == 0
        // the overflow series has finished: to the remainder if any
        if(remainder) {
          timer->CCMP = remainder;
          if(timer->CNT < remainder) return;
          // remainder is so short: already passed !
          timer->CCMP = countsPerOverflow;
        }
        // no remainder series: reset the overflow counter and do the callback
        overflowCounter = overflowCounts;
      }
      (*isrCallback)();
    }

private:
    TCB_t * timer = &TCB0;
    long overflowCounts = 0;
    long remainder = 10;
    long overflowCounter = 0;
    unsigned long countsPerOverflow = TCB_RESOLUTION - 1;
    void (*isrCallback)();
    static void isrDefaultUnused();
    unsigned long maxTimeWithoutOverflow;
  
}; // EveryTimerB

extern EveryTimerB TimerB2;

#endif // ARDUINO_ARCH_MEGAAVR
#endif // EveryTimerB_h_
