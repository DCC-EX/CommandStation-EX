/*
 *  © 2022 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021-2022 Harald Barth
 *  © 2021 Fred Decker
 *  © 2021 Chris Harlow
 *  © 2021 David Cutting
 *  All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

// ATTENTION: this file only compiles on a SAMD21 based board
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_SAMD

#include "DCCTimer.h"
#include <wiring_private.h>

INTERRUPT_CALLBACK interruptHandler=0;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  noInterrupts();
  // Timer setup - setup clock sources first
  REG_GCLK_GENDIV =   GCLK_GENDIV_DIV(1) |            // Divide 48MHz by 1
                      GCLK_GENDIV_ID(4);              // Apply to GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
                
  REG_GCLK_GENCTRL =  GCLK_GENCTRL_GENEN |            // Enable GCLK
                      GCLK_GENCTRL_SRC_DFLL48M |      // Set the 48MHz clock source
                      GCLK_GENCTRL_ID(4);             // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
            
  REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_CLKEN |            // Enable generic clock
                      4 << GCLK_CLKCTRL_GEN_Pos |     // Apply to GCLK4
                      GCLK_CLKCTRL_ID_TCC0_TCC1;      // Feed GCLK to TCC0/1
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Assume we're using TCC0... as we're bit-bashing the DCC waveform output pins anyway
  // for "normal accuracy" DCC waveform generation. For high accuracy we're going to need
  // to a good deal more. The TCC waveform output pins are mux'd on the SAMD, and output
  // pins for each TCC are only available on certain pins
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;     // Select NPWM as waveform
  while (TCC0->SYNCBUSY.bit.WAVE);            // Wait for sync

  // Set the frequency
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);
  TCC0->PER.reg = CLOCK_CYCLES * 2;
  while (TCC0->SYNCBUSY.bit.PER);

  // Start the timer
  TCC0->CTRLA.bit.ENABLE = 1;
  while (TCC0->SYNCBUSY.bit.ENABLE);

  // Set the interrupt condition, priority and enable it in the NVIC
  TCC0->INTENSET.reg = TCC_INTENSET_OVF;      // Only interrupt on overflow
  int USBprio = NVIC_GetPriority((IRQn_Type) USB_IRQn);  // Fetch the USB priority
  NVIC_SetPriority((IRQn_Type)TCC0_IRQn, USBprio);  // Match the USB priority
//  NVIC_SetPriority((IRQn_Type)TCC0_IRQn, 0);  // Make this highest priority
  NVIC_EnableIRQ((IRQn_Type)TCC0_IRQn);       // Enable the interrupt
  interrupts();
}

void DCCTimer::startRailcomTimer(byte brakePin) {
  // TODO: for intended operation see DCCTimerAVR.cpp
  (void) brakePin; 
}

void DCCTimer::ackRailcomTimer() {
  // TODO: for intended operation see DCCTimerAVR.cpp
}

// Timer IRQ handlers replace the dummy handlers (in cortex_handlers)
// copied from rf24 branch
void TCC0_Handler() {
    if(TCC0->INTFLAG.bit.OVF) {
        TCC0->INTFLAG.bit.OVF = 1; // writing a 1 clears the flag
        interruptHandler();
    }
}

void TCC1_Handler() {
    if(TCC1->INTFLAG.bit.OVF) {
        TCC1->INTFLAG.bit.OVF = 1; // writing a 1 clears the flag
        interruptHandler();
    }
}

void TCC2_Handler() {
    if(TCC2->INTFLAG.bit.OVF) {
        TCC2->INTFLAG.bit.OVF = 1; // writing a 1 clears the flag
        interruptHandler();
    }
}


bool DCCTimer::isPWMPin(byte pin) {
  //TODO: SAMD whilst this call to digitalPinHasPWM will reveal which pins can do PWM,
  //      there's no support yet for High Accuracy, so for now return false
  //  return digitalPinHasPWM(pin);
  return false;
}

void DCCTimer::setPWM(byte pin, bool high) {
    // TODO: High Accuracy mode is not supported as yet, and may never need to be
    (void) pin;
    (void) high;
}

void DCCTimer::clearPWM() {
  return;
}

void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  volatile uint32_t *serno1 = (volatile uint32_t *)0x0080A00C;
  volatile uint32_t *serno2 = (volatile uint32_t *)0x0080A040;
//  volatile uint32_t *serno3 = (volatile uint32_t *)0x0080A044;
//  volatile uint32_t *serno4 = (volatile uint32_t *)0x0080A048;

  volatile uint32_t m1 = *serno1;
  volatile uint32_t m2 = *serno2;
  mac[0] = m1 >> 8;
  mac[1] = m1 >> 0;
  mac[2] = m2 >> 24;
  mac[3] = m2 >> 16;
  mac[4] = m2 >> 8;
  mac[5] = m2 >> 0;
}

volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = freeMemory();
  interrupts();
  return retval;
}

extern "C" char* sbrk(int incr);

int DCCTimer::freeMemory() {
  char top;
  return (int)(&top - reinterpret_cast<char *>(sbrk(0)));
}

void DCCTimer::reset() {
   __disable_irq();
    NVIC_SystemReset();
    while(true) {};
}

#define NUM_ADC_INPUTS NUM_ANALOG_INPUTS

uint16_t ADCee::usedpins = 0;
int * ADCee::analogvals = NULL;

int ADCee::init(uint8_t pin) {
  uint8_t id = pin - A0;
  int value = 0;

  if (id > NUM_ADC_INPUTS)
    return -1023;

  // Permanently configure SAMD IO MUX for that pin
  pinPeripheral(pin, PIO_ANALOG);
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input

  // Start conversion
  ADC->SWTRIG.bit.START = 1;

  // Wait for the conversion to be ready
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete

  // Read the value
  value = ADC->RESULT.reg;

  if (analogvals == NULL)
    analogvals = (int *)calloc(NUM_ADC_INPUTS+1, sizeof(int));
  analogvals[id] = value;
  usedpins |= (1<<id);

  return value;
}

int16_t ADCee::ADCmax() {
  return 4095;
}

/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  uint8_t id = pin - A0;
  if ((usedpins & (1<<id) ) == 0)
    return -1023;
  // we do not need to check (analogvals == NULL)
  // because usedpins would still be 0 in that case
  return analogvals[id];
}
/*
 * Scan function that is called from interrupt
 */
#pragma GCC push_options
#pragma GCC optimize ("-O3")
void ADCee::scan() {
  static uint8_t id = 0;        // id and mask are the same thing but it is faster to 
  static uint16_t mask = 1;  // increment and shift instead to calculate mask from id
  static bool waiting = false;

  if (waiting) {
    // look if we have a result
    if (ADC->INTFLAG.bit.RESRDY == 0)
      return; // no result, continue to wait
    // found value
    analogvals[id] = ADC->RESULT.reg;
    // advance at least one track
    // for scope debug TrackManager::track[1]->setBrake(0);
    waiting = false;
    id++;
    mask = mask << 1;
    if (id == NUM_ADC_INPUTS+1) {
      id = 0;
      mask = 1;
    }
  }
  if (!waiting) {
    if (usedpins == 0) // otherwise we would loop forever
      return;
    // look for a valid track to sample or until we are around
    while (true) {
      if (mask  & usedpins) {
    	  // start new ADC aquire on id
        ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[id + A0].ulADCChannelNumber; // Selection for the positive ADC input
          // Start conversion
        ADC->SWTRIG.bit.START = 1;
	      // for scope debug TrackManager::track[1]->setBrake(1);
	      waiting = true;
	      return;
      }
      id++;
      mask = mask << 1;
      if (id == NUM_ADC_INPUTS+1) {
	      id = 0;
	      mask = 1;
      }
    }
  }
}
#pragma GCC pop_options

void ADCee::begin() {
  noInterrupts();
  // Set up ADC to do faster reads... default for Arduino Zero platform configs is 436uS,
  // and we need sub-58uS. This code sets it to a read speed of around 5-6uS, and enables
  // 12-bit mode
  // Reconfigure ADC
  ADC->CTRLA.bit.ENABLE = 0;                      // disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );         // wait for synchronization

  ADC->CTRLB.reg &= 0b1111100011001111;           // mask PRESCALER and RESSEL bits
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV64 |   // divide Clock by 16
                    ADC_CTRLB_RESSEL_12BIT;       // Result 12 bits, 10 bits possible
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // take 1 sample at a time
                     ADC_AVGCTRL_ADJRES(0x00ul);  // adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00ul;                     // sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                      // enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );         // wait for synchronization
  interrupts();
}
#endif
