/*
 *  © 2021 Mike S
 *  © 2021-2023 Harald Barth
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

// ATTENTION: this file only compiles on a UNO or MEGA
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_AVR
#include <avr/boot.h> 
#include <avr/wdt.h>
#include "DCCTimer.h"
#ifdef DEBUG_ADC
#include "TrackManager.h"
#endif
INTERRUPT_CALLBACK interruptHandler=0;

  // Arduino nano, uno, mega etc
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define TIMER1_A_PIN   11
    #define TIMER1_B_PIN   12
    #define TIMER1_C_PIN   13
    #define TIMER2_A_PIN   10
    #define TIMER2_B_PIN   9
    
#else
   #define TIMER1_A_PIN   9
   #define TIMER1_B_PIN   10
#endif

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler=callback;
    noInterrupts();
    TCCR1A = 0;
    ICR1 = CLOCK_CYCLES;
    TCNT1 = 0;   
    TCCR1B = _BV(WGM13) | _BV(CS10);     // Mode 8, clock select 1
    TIMSK1 = _BV(TOIE1); // Enable Software interrupt
    interrupts();
  }


void DCCTimer::startRailcomTimer(byte brakePin) {
  /* The Railcom timer is started in such a way that it 
     - First triggers 28uS after the last TIMER1 tick. 
       This provides an accurate offset (in High Accuracy mode)
       for the start of the Railcom cutout.
     - Sets the Railcom pin high at first tick, 
       because its been setup with 100% PWM duty cycle.
        
     - Cycles at 436uS so the second tick is the 
        correct distance from the cutout.
        
     - Waveform code is responsible for altering the PWM 
       duty cycle to 0% any time between the first and last tick.
       (there will be 7 DCC timer1 ticks in which to do this.)
    
    */
  (void) brakePin; // Ignored... works on pin 9 only 
  const int cutoutDuration = 430; // Desired interval in microseconds
  
  // Set up Timer2 for CTC mode (Clear Timer on Compare Match)
  TCCR2A = 0; // Clear Timer2 control register A
  TCCR2B = 0; // Clear Timer2 control register B
  TCNT2 = 0;  // Initialize Timer2 counter value to 0
   // Configure Phase and Frequency Correct PWM mode
   TCCR2A =  (1 << COM2B1); // enable pwm on pin 9
   TCCR2A |= (1 << WGM20);
  
   
  // Set Timer 2 prescaler to 32
  TCCR2B = (1 << CS21) | (1 << CS20); // 32 prescaler

  // Set the compare match value for desired interval
  OCR2A = (F_CPU / 1000000) * cutoutDuration / 64 - 1;

  // Calculate the compare match value for desired duty cycle
  OCR2B = OCR2A+1;  // set duty cycle to 100%= OCR2A)

  // Enable Timer2 output on pin 9 (OC2B)
  DDRB |= (1 << DDB1);
  // TODO Fudge TCNT2 to sync with last tcnt1 tick + 28uS

 // Previous TIMER1 Tick was at rising end-of-packet bit
 // Cutout starts half way through first preamble
 // that is 2.5 * 58uS later.
 // TCNT1 ticks 8 times / microsecond
 // auto microsendsToFirstRailcomTick=(58+58+29)-(TCNT1/8);
  // set the railcom timer counter allowing for phase-correct

  // CHris's NOTE: 
  // I dont kniow quite how this calculation works out but
  // it does seems to get a good answer. 

  TCNT2=193 + (ICR1 - TCNT1)/8;
}

void DCCTimer::ackRailcomTimer() {
  OCR2B= 0x00;  // brake pin pwm duty cycle 0 at next tick
}


// ISR called by timer interrupt every 58uS
  ISR(TIMER1_OVF_vect){ interruptHandler(); }

// Alternative pin manipulation via PWM control.
  bool DCCTimer::isPWMPin(byte pin) {
       return pin==TIMER1_A_PIN 
           || pin==TIMER1_B_PIN
       #ifdef TIMER1_C_PIN 
           || pin==TIMER1_C_PIN
       #endif       
       ;
  }

 void DCCTimer::setPWM(byte pin, bool high) {
    if (pin==TIMER1_A_PIN) {
      TCCR1A |= _BV(COM1A1);
      OCR1A= high?1024:0;
    }
    else if (pin==TIMER1_B_PIN) { 
      TCCR1A |= _BV(COM1B1);
      OCR1B= high?1024:0;
    }
 #ifdef TIMER1_C_PIN 
    else if (pin==TIMER1_C_PIN) { 
      TCCR1A |= _BV(COM1C1);
      OCR1C= high?1024:0;
    }
 #endif       
 }

void DCCTimer::clearPWM() {
  TCCR1A= 0;
}

  void DCCTimer::getSimulatedMacAddress(byte mac[6]) {
    for (byte i=0; i<6; i++) {
      // take the fist 3 and last 3 of the serial.
      // the first 5 of 8 are at 0x0E to 0x013
      // the last  3 of 8 are at 0x15 to 0x017
      mac[i]=boot_signature_byte_get(0x0E + i + (i>2? 4 : 0));
    }
    mac[0] &= 0xFE;
    mac[0] |= 0x02;
  }


volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = minimum_free_memory;
  interrupts();
  return retval;
}

extern char *__brkval;
extern char *__malloc_heap_start;

int DCCTimer::freeMemory() {
  char top;
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
}

void DCCTimer::reset() {
  wdt_enable( WDTO_15MS); // set Arduino watchdog timer for 15ms 
  delay(50);            // wait for the prescaller time to expire

}

#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
#define NUM_ADC_INPUTS 16
#else
#define NUM_ADC_INPUTS 8
#endif
uint16_t ADCee::usedpins = 0;
uint8_t ADCee::highestPin = 0;
int * ADCee::analogvals = NULL;
static bool ADCusesHighPort = false;

/*
 * Register a new pin to be scanned
 * Returns current reading of pin and
 * stores that as well
 */
int ADCee::init(uint8_t pin) {
  uint8_t id = pin - A0;
  if (id >= NUM_ADC_INPUTS)
    return -1023;
  if (id > 7)
    ADCusesHighPort = true;
  pinMode(pin, INPUT);
  int value = analogRead(pin);
  if (analogvals == NULL)
    analogvals = (int *)calloc(NUM_ADC_INPUTS, sizeof(int));
  analogvals[id] = value;
  usedpins |= (1<<id);
  if (id > highestPin) highestPin = id;
  return value;
}
int16_t ADCee::ADCmax() {
  return 1023;
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
  if (!fromISR) noInterrupts();
  int a = analogvals[id];
  if (!fromISR) interrupts();
  return a;
}
/*
 * Scan function that is called from interrupt
 */
#pragma GCC push_options
#pragma GCC optimize ("-O3")
void ADCee::scan() {
  static byte id = 0;       // id and mask are the same thing but it is faster to
  static uint16_t mask = 1; // increment and shift instead to calculate mask from id
  static bool waiting = false;

  if (waiting) {
    // look if we have a result
    byte low, high;
    if (bit_is_set(ADCSRA, ADSC))
      return; // no result, continue to wait
    // found value
    low = ADCL; //must read low before high
    high = ADCH;
    bitSet(ADCSRA, ADIF);
    analogvals[id] = (high << 8) | low;
    // advance at least one track
#ifdef DEBUG_ADC
    if (id == 1) TrackManager::track[1]->setBrake(0);
#endif
    waiting = false;
    id++;
    mask = mask << 1;
    if (id > highestPin) {
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
#if defined(ADCSRB) && defined(MUX5)
	if (ADCusesHighPort) { // if we ever have started to use high pins)
	  if (id > 7)          // if we use a high ADC pin
	    bitSet(ADCSRB, MUX5); // set MUX5 bit
	  else
	    bitClear(ADCSRB, MUX5);
	}
#endif
	ADMUX=(1<<REFS0)|(id & 0x07); //select AVCC as reference and set MUX
	bitSet(ADCSRA,ADSC); // start conversion
#ifdef DEBUG_ADC
	if (id == 1) TrackManager::track[1]->setBrake(1);
#endif
	waiting = true;
	return;
      }
      id++;
      mask = mask << 1;
      if (id > highestPin) {
      	id = 0;
	      mask = 1;
      }
    }
  }
}
#pragma GCC pop_options

void ADCee::begin() {
  noInterrupts();
  // ADCSRA = (ADCSRA & 0b11111000) | 0b00000100;   // speed up analogRead sample time
  // Set up ADC for free running mode
  ADMUX=(1<<REFS0); //select AVCC as reference. We set MUX later
  ADCSRA = (1<<ADEN)|(1 << ADPS2); // ADPS2 means divisor 32 and 16Mhz/32=500kHz.
  //bitSet(ADCSRA, ADSC); //do not start the ADC yet. Done when we have set the MUX
  interrupts();
}
#endif
