/*
 *  © 2023 Neil McKechnie
 *  © 2022 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021 Harald Barth
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

// ATTENTION: this file only compiles on a STM32 based boards
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.
#ifdef ARDUINO_ARCH_STM32

#include "DCCTimer.h"

#if defined(ARDUINO_NUCLEO_F411RE)
// Nucleo-64 boards don't have Serial1 defined by default
HardwareSerial Serial1(PB7, PA15);  // Rx=PB7, Tx=PA15 -- CN7 pins 17 and 21 - F411RE
// Serial2 is defined to use USART2 by default, but is in fact used as the diag console
// via the debugger on the Nucleo-64. It is therefore unavailable for other DCC-EX uses like WiFi, DFPlayer, etc.
// Let's define Serial6 as an additional serial port (the only other option for the Nucleo-64s)
HardwareSerial Serial6(PA12, PA11);  // Rx=PA12, Tx=PA11 -- CN10 pins 12 and 14 - F411RE
#elif defined(ARDUINO_NUCLEO_F446RE)
// Nucleo-64 boards don't have Serial1 defined by default
HardwareSerial Serial1(PA10, PB6);  // Rx=PA10, Tx=PB6 -- CN10 pins 33 and 17 - F446RE 
// Serial2 is defined to use USART2 by default, but is in fact used as the diag console
// via the debugger on the Nucleo-64. It is therefore unavailable for other DCC-EX uses like WiFi, DFPlayer, etc.
#elif defined(ARDUINO_NUCLEO_F412ZG) || defined(ARDUINO_NUCLEO_F429ZI) || defined(ARDUINO_NUCLEO_F446ZE)
// Nucleo-144 boards don't have Serial1 defined by default
HardwareSerial Serial1(PG9, PG14);  // Rx=PG9, Tx=PG14 -- D0, D1 - F412ZG/F446ZE
#else
#warning Serial1 not defined
#endif

INTERRUPT_CALLBACK interruptHandler=0;
// Let's use STM32's timer #2 which supports hardware pulse generation on pins D3 and D6 
// (accurate timing, independent of the latency of interrupt handling).
// Pin D3 is driven by TIM2 channel 2 and D6 is TIM2 channel 3.
HardwareTimer timer(TIM2);

// Timer IRQ handler
void Timer_Handler() {
  interruptHandler();
}

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  noInterrupts();

  // adc_set_sample_rate(ADC_SAMPLETIME_480CYCLES);
  timer.pause();
  timer.setPrescaleFactor(1);
  timer.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
  timer.attachInterrupt(Timer_Handler);
  timer.refresh();
  timer.resume();

  interrupts();
}

bool DCCTimer::isPWMPin(byte pin) {
  // Timer 2 Channel 2 controls pin D3, and Timer2 Channel 3 controls D6.
  //  Enable the appropriate timer channel.
  switch (pin) {
    case 3:
      timer.setMode(2, TIMER_OUTPUT_COMPARE_INACTIVE, D3);
      return true;
    case 6:
      timer.setMode(3, TIMER_OUTPUT_COMPARE_INACTIVE, D6);
      return true;
    default:
      return false;
  }
}

void DCCTimer::setPWM(byte pin, bool high) {
  // Set the timer so that, at the next counter overflow, the requested
  // pin state is activated automatically before the interrupt code runs.
  switch (pin) {
    case 3:
      if (high) 
        TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | TIM_CCMR1_OC2M_0;
      else
        TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC2M_Msk) | TIM_CCMR1_OC2M_1;
      break;
    case 6:
      if (high) 
        TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | TIM_CCMR2_OC3M_0;
      else
        TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M_Msk) | TIM_CCMR2_OC3M_1;
      break;
  }   
}

void DCCTimer::clearPWM() {
  timer.setMode(2, TIMER_OUTPUT_COMPARE_INACTIVE, NC);
  timer.setMode(3, TIMER_OUTPUT_COMPARE_INACTIVE, NC);  
}

void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  volatile uint32_t *serno1 = (volatile uint32_t *)0x1FFF7A10;
  volatile uint32_t *serno2 = (volatile uint32_t *)0x1FFF7A14;
  // volatile uint32_t *serno3 = (volatile uint32_t *)0x1FFF7A18;

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

// TODO: may need to use uint32_t on STMF4xx variants with > 16 analog inputs!
uint16_t ADCee::usedpins = 0;
int * ADCee::analogvals = NULL;
uint32_t * analogchans = NULL;
bool adc1configured = false;

int16_t ADCee::ADCmax() {
  return 4095;
}

int ADCee::init(uint8_t pin) {
  uint id = pin - A0;
  int value = 0;
  PinName stmpin = digitalPin[analogInputPin[id]];
  uint32_t stmgpio = stmpin / 16; // 16-bits per GPIO port group on STM32
  uint32_t adcchan =  STM_PIN_CHANNEL(pinmap_function(stmpin, PinMap_ADC)); // find ADC channel (only valid for ADC1!)
  GPIO_TypeDef * gpioBase;

  // Port config - find which port we're on and power it up
  switch(stmgpio) {
    case 0x00:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Power up PORTA
        gpioBase = GPIOA;
        break;
    case 0x01:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //Power up PORTB
        gpioBase = GPIOB;
        break;
    case 0x02:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Power up PORTC
        gpioBase = GPIOC;
        break;
  }

  // Set pin mux mode to analog input
  gpioBase->MODER |= (0b011 << (stmpin << 1)); // Set pin mux to analog mode

  // Set the sampling rate for that analog input
  if (adcchan < 10)
    ADC1->SMPR2 |= (0b111 << (adcchan * 3)); // Channel sampling rate 480 cycles
  else
    ADC1->SMPR1 |= (0b111 << ((adcchan - 10) * 3)); // Channel sampling rate 480 cycles

  // Read the inital ADC value for this analog input
  ADC1->SQR3 = adcchan;           // 1st conversion in regular sequence
  ADC1->CR2 |= (1 << 30);         // Start 1st conversion SWSTART
  while(!(ADC1->SR & (1 << 1)));  // Wait until conversion is complete
  value = ADC1->DR;               // Read value from register

  if (analogvals == NULL)
  {
    analogvals = (int *)calloc(NUM_ADC_INPUTS+1, sizeof(int));
    analogchans = (uint32_t *)calloc(NUM_ADC_INPUTS+1, sizeof(uint32_t));
  }
  analogvals[id] = value;     // Store sampled value
  analogchans[id] = adcchan;  // Keep track of which ADC channel is used for reading this pin
  usedpins |= (1 << id);      // This pin is now ready

  return value;
}

/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  uint8_t id = pin - A0;
  // Was this pin initialised yet?
  if ((usedpins & (1<<id) ) == 0)
    return -1023;
  // We do not need to check (analogvals == NULL)
  // because usedpins would still be 0 in that case
  return analogvals[id];
}

/*
 * Scan function that is called from interrupt
 */
#pragma GCC push_options
#pragma GCC optimize ("-O3")
void ADCee::scan() {
  static uint id = 0;        // id and mask are the same thing but it is faster to 
  static uint16_t mask = 1;  // increment and shift instead to calculate mask from id
  static bool waiting = false;

  if (waiting) {
    // look if we have a result
    if (!(ADC1->SR & (1 << 1)))
      return; // no result, continue to wait
    // found value
    analogvals[id] = ADC1->DR;
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
        ADC1->SQR3 = analogchans[id]; //1st conversion in regular sequence
        ADC1->CR2 |= (1 << 30); //Start 1st conversion SWSTART
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
  //ADC1 config sequence
  // TODO: currently defaults to ADC1, may need more to handle other members of STM32F4xx family
  RCC->APB2ENR |= (1 << 8); //Enable ADC1 clock (Bit8) 
  // Set ADC prescaler - DIV8 ~ 40ms, DIV6 ~ 30ms, DIV4 ~ 20ms, DIV2 ~ 11ms
  ADC->CCR = (0 << 16); // Set prescaler 0=DIV2, 1=DIV4, 2=DIV6, 3=DIV8
  ADC1->CR1 &= ~(1 << 8); //SCAN mode disabled (Bit8)
  ADC1->CR1 &= ~(3 << 24); //12bit resolution (Bit24,25 0b00)
  ADC1->SQR1 = (1 << 20); //Set number of conversions projected (L[3:0] 0b0001) -> 1 conversion
  ADC1->CR2 &= ~(1 << 1); //Single conversion
  ADC1->CR2 &= ~(1 << 11); //Right alignment of data bits bit12....bit0
  ADC1->SQR1 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->SQR2 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->SQR3 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->CR2 |= (1 << 0); // Switch on ADC1
  interrupts();
}
#endif