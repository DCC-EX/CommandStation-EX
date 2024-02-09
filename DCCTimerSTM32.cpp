/*
 *  © 2023 Neil McKechnie
 *  © 2022-2023 Paul M. Antoine
 *  © 2021 Mike S
 *  © 2021, 2023 Harald Barth
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
#ifdef DEBUG_ADC
#include "TrackManager.h"
#endif
#include "DIAG.h"

#if defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_F411RE)
// Nucleo-64 boards don't have additional serial ports defined by default
HardwareSerial Serial1(PB7, PA15);  // Rx=PB7, Tx=PA15 -- CN7 pins 17 and 21 - F411RE
// Serial2 is defined to use USART2 by default, but is in fact used as the diag console
// via the debugger on the Nucleo-64. It is therefore unavailable for other DCC-EX uses like WiFi, DFPlayer, etc.
// Let's define Serial6 as an additional serial port (the only other option for the Nucleo-64s)
HardwareSerial Serial6(PA12, PA11);  // Rx=PA12, Tx=PA11 -- CN10 pins 12 and 14 - F411RE
#elif defined(ARDUINO_NUCLEO_F446RE)
// Nucleo-64 boards don't have additional serial ports defined by default
// On the F446RE, Serial1 isn't really useable as it's Rx/Tx pair sit on already used D2/D10 pins
// HardwareSerial Serial1(PA10, PB6);  // Rx=PA10 (D2), Tx=PB6 (D10) -- CN10 pins 17 and 9 - F446RE 
// Serial2 is defined to use USART2 by default, but is in fact used as the diag console
// via the debugger on the Nucleo-64. It is therefore unavailable for other DCC-EX uses like WiFi, DFPlayer, etc.
// On the F446RE, Serial3 and Serial5 are easy to use:
HardwareSerial Serial3(PC11, PC10);  // Rx=PC11, Tx=PC10 -- USART3 - F446RE
HardwareSerial Serial5(PD2, PC12);  // Rx=PD2, Tx=PC12 -- UART5 - F446RE
// On the F446RE, Serial4 and Serial6 also use pins we can't readily map while using the Arduino pins
#elif defined(ARDUINO_NUCLEO_F412ZG) || defined(ARDUINO_NUCLEO_F413ZH) || defined(ARDUINO_NUCLEO_F446ZE) || \
      defined(ARDUINO_NUCLEO_F429ZI) || defined(ARDUINO_NUCLEO_F439ZI)
// Nucleo-144 boards don't have Serial1 defined by default
HardwareSerial Serial6(PG9, PG14);  // Rx=PG9, Tx=PG14 -- USART6
HardwareSerial Serial5(PD2, PC12);  // Rx=PD2, Tx=PC12 -- UART5
#if !defined(ARDUINO_NUCLEO_F412ZG)
  HardwareSerial Serial2(PD6, PD5);  // Rx=PD6, Tx=PD5 -- UART5
#endif  
// Serial3 is defined to use USART3 by default, but is in fact used as the diag console
// via the debugger on the Nucleo-144. It is therefore unavailable for other DCC-EX uses like WiFi, DFPlayer, etc.
#else
#error STM32 board selected is not yet explicitly supported - so Serial1 peripheral is not defined
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
// Experimental code for High Accuracy (HA) DCC Signal mode
// Warning - use of TIM2 and TIM3 can affect the use of analogWrite() function on certain pins,
// which is used by the DC motor types.
///////////////////////////////////////////////////////////////////////////////////////////////

// INTERRUPT_CALLBACK interruptHandler=0;
// // Let's use STM32's timer #2 which supports hardware pulse generation on pin D13.
// // Also, timer #3 will do hardware pulses on pin D12. This gives
// // accurate timing, independent of the latency of interrupt handling.
// // We only need to interrupt on one of these (TIM2), the other will just generate
// // pulses.
// HardwareTimer timer(TIM2);
// HardwareTimer timerAux(TIM3);
// static bool tim2ModeHA = false;
// static bool tim3ModeHA = false;

// // Timer IRQ handler
// void Timer_Handler() {
//   interruptHandler();
// }

// void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
//   interruptHandler=callback;
//   noInterrupts();

//   // adc_set_sample_rate(ADC_SAMPLETIME_480CYCLES);
//   timer.pause();
//   timerAux.pause();
//   timer.setPrescaleFactor(1);
//   timer.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
//   timer.attachInterrupt(Timer_Handler);
//   timer.refresh();
//   timerAux.setPrescaleFactor(1);
//   timerAux.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
//   timerAux.refresh();
  
//   timer.resume();
//   timerAux.resume();

//   interrupts();
// }

// bool DCCTimer::isPWMPin(byte pin) {
//   // Timer 2 Channel 1 controls pin D13, and Timer3 Channel 1 controls D12.
//   //  Enable the appropriate timer channel.
//   switch (pin) {
//     case 12:
//       return true;
//     case 13:
//       return true;
//     default:
//       return false;
//   }
// }

// void DCCTimer::setPWM(byte pin, bool high) {
//   // Set the timer so that, at the next counter overflow, the requested
//   // pin state is activated automatically before the interrupt code runs.
//   // TIM2 is timer, TIM3 is timerAux.
//   switch (pin) {
//     case 12:
//       if (!tim3ModeHA) {
//         timerAux.setMode(1, TIMER_OUTPUT_COMPARE_INACTIVE, D12);
//         tim3ModeHA = true;
//       }
//       if (high) 
//         TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_CCMR1_OC1M_0;
//       else
//         TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_CCMR1_OC1M_1;
//       break;
//     case 13:
//       if (!tim2ModeHA) {
//         timer.setMode(1, TIMER_OUTPUT_COMPARE_INACTIVE, D13);
//         tim2ModeHA = true;
//       }
//       if (high) 
//         TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_CCMR1_OC1M_0;
//       else
//         TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | TIM_CCMR1_OC1M_1;
//       break;
//   }   
// }

// void DCCTimer::clearPWM() {
//   timer.setMode(1, TIMER_OUTPUT_COMPARE_INACTIVE, NC);
//   tim2ModeHA = false;
//   timerAux.setMode(1, TIMER_OUTPUT_COMPARE_INACTIVE, NC);  
//   tim3ModeHA = false;
// }
///////////////////////////////////////////////////////////////////////////////////////////////

INTERRUPT_CALLBACK interruptHandler=0;

// On STM32F4xx models that have them, Timers 6 and 7 have no PWM output capability,
// so are good choices for general timer duties - they are used for tone and servo
// in stm32duino so we shall usurp those as DCC-EX doesn't use tone or servo libs.
// NB: the F401, F410 and F411 do **not** have Timer 6 or 7, so we use Timer 11
#ifndef DCC_EX_TIMER
#if defined(TIM6)
#define DCC_EX_TIMER TIM6
#elif defined(TIM7)
#define DCC_EX_TIMER TIM7
#elif defined(TIM11)
#define DCC_EX_TIMER TIM11
#else
#warning This STM32F4XX variant does not have Timers 6,7 or 11!!
#endif
#endif // ifndef DCC_EX_TIMER

HardwareTimer dcctimer(DCC_EX_TIMER);
void DCCTimer_Handler() __attribute__((interrupt));

// Timer IRQ handler
void DCCTimer_Handler() {
  interruptHandler();
}

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  noInterrupts();

  dcctimer.pause();
  dcctimer.setPrescaleFactor(1);
//  timer.setOverflow(CLOCK_CYCLES * 2);
  dcctimer.setOverflow(DCC_SIGNAL_TIME, MICROSEC_FORMAT);
  // dcctimer.attachInterrupt(Timer11_Handler);
  dcctimer.attachInterrupt(DCCTimer_Handler);
  dcctimer.setInterruptPriority(0, 0); // Set highest preemptive priority!
  dcctimer.refresh();
  dcctimer.resume();

  interrupts();
}

void DCCTimer::startRailcomTimer(byte brakePin) {
  // TODO: for intended operation see DCCTimerAVR.cpp
  (void) brakePin; 
}

void DCCTimer::ackRailcomTimer() {
  // TODO: for intended operation see DCCTimerAVR.cpp
}

bool DCCTimer::isPWMPin(byte pin) {
  //TODO: STM32 whilst this call to digitalPinHasPWM will reveal which pins can do PWM,
  //      there's no support yet for High Accuracy, so for now return false
  //  return digitalPinHasPWM(pin);
  (void) pin;
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
  volatile uint32_t *serno1 = (volatile uint32_t *)UID_BASE;
  volatile uint32_t *serno2 = (volatile uint32_t *)UID_BASE+4;
  // volatile uint32_t *serno3 = (volatile uint32_t *)UID_BASE+8;

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

// TODO: rationalise the size of these... could really use sparse arrays etc.
static HardwareTimer * pin_timer[100] = {0};
static uint32_t channel_frequency[100] = {0};
static uint32_t pin_channel[100] = {0};

// Using the HardwareTimer library API included in stm32duino core to handle PWM duties
// TODO: in order to use the HA code above which Neil kindly wrote, we may have to do something more
// sophisticated about detecting any clash between the timer we'd like to use for PWM and the ones
// currently used for HA so they don't interfere with one another. For now we'll just make PWM
// work well... then work backwards to integrate with HA mode if we can.
void DCCTimer::DCCEXanalogWriteFrequency(uint8_t pin, uint32_t frequency)
{
  if (pin_timer[pin] == NULL) {
    // Automatically retrieve TIM instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
    if (Instance == NULL) {
      // We shouldn't get here (famous last words) as it ought to have been caught by brakeCanPWM()!
      DIAG(F("DCCEXanalogWriteFrequency::Pin %d has no PWM function!"), pin);
      return;
    }
    pin_channel[pin] = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

    // Instantiate HardwareTimer object. Thanks to 'new' instantiation,
    // HardwareTimer is not destructed when setup function is finished.
    pin_timer[pin] = new HardwareTimer(Instance);
    // Configure and start PWM
    // MyTim->setPWM(channel, pin, 5, 10, NULL, NULL); // No callback required, we can simplify the function call
    if (pin_timer[pin] != NULL)
    {
      pin_timer[pin]->setPWM(pin_channel[pin], pin, frequency, 0); // set frequency in Hertz, 0% dutycycle
      DIAG(F("DCCEXanalogWriteFrequency::Pin %d on Timer %d, frequency %d"), pin, pin_channel[pin], frequency);
    }
    else
      DIAG(F("DCCEXanalogWriteFrequency::failed to allocate HardwareTimer instance!"));
  }
  else
  {
    // Frequency change request
    if (frequency != channel_frequency[pin])
    {
      pinmap_pinout(digitalPinToPinName(pin), PinMap_TIM); // ensure the pin has been configured!
      pin_timer[pin]->setOverflow(frequency, HERTZ_FORMAT); // Just change the frequency if it's already running!
      DIAG(F("DCCEXanalogWriteFrequency::setting frequency to %d"), frequency);
    }
  }
  channel_frequency[pin] = frequency;
  return;
}

void DCCTimer::DCCEXanalogWrite(uint8_t pin, int value) {
    // Calculate percentage duty cycle from value given
    uint32_t duty_cycle = (value * 100 / 256) + 1;
    if (pin_timer[pin] != NULL) {
      // if (duty_cycle == 100)
      // {
      //   pin_timer[pin]->pauseChannel(pin_channel[pin]);
      //   DIAG(F("DCCEXanalogWrite::Pausing timer channel on pin %d"), pin);
      // }
      // else
      // {
        pinmap_pinout(digitalPinToPinName(pin), PinMap_TIM); // ensure the pin has been configured!
        // pin_timer[pin]->resumeChannel(pin_channel[pin]);
        pin_timer[pin]->setCaptureCompare(pin_channel[pin], duty_cycle, PERCENT_COMPARE_FORMAT); // DCC_EX_PWM_FREQ Hertz, duty_cycle% dutycycle
        DIAG(F("DCCEXanalogWrite::Pin %d, value %d, duty cycle %d"), pin, value, duty_cycle);
      // }
    }
    else
      DIAG(F("DCCEXanalogWrite::Pin %d is not configured for PWM!"), pin);
}


// Now we can handle more ADCs, maybe this works!
#define NUM_ADC_INPUTS NUM_ANALOG_INPUTS

uint32_t ADCee::usedpins = 0;         // Max of 32 ADC input channels!
uint8_t ADCee::highestPin = 0;        // Highest pin to scan
int * ADCee::analogvals = NULL;       // Array of analog values last captured
uint32_t * analogchans = NULL;        // Array of channel numbers to be scanned
// bool adc1configured = false;
ADC_TypeDef * * adcchans = NULL;      // Array to capture which ADC is each input channel on

int16_t ADCee::ADCmax()
{
    return 4095;
}

int ADCee::init(uint8_t pin) {

  int value = 0;
  PinName stmpin = analogInputToPinName(pin);
  if (stmpin == NC) // do not continue if this is not an analog pin at all
    return -1024;   // some silly value as error

  uint32_t stmgpio = STM_PORT(stmpin); // converts to the GPIO port (16-bits per port group on STM32)
  uint32_t adcchan =  STM_PIN_CHANNEL(pinmap_function(stmpin, PinMap_ADC)); // find ADC input channel
  ADC_TypeDef *adc = (ADC_TypeDef *)pinmap_find_peripheral(stmpin, PinMap_ADC); // find which ADC this pin is on ADC1/2/3 etc.
  int adcnum = 1;
  if (adc == ADC1)
    DIAG(F("ADCee::init(): found pin %d on ADC1"), pin);
// Checking for ADC2 and ADC3 being defined helps cater for more variants later
#if defined(ADC2)
  else if (adc == ADC2)
  {
    DIAG(F("ADCee::init(): found pin %d on ADC2"), pin);
    adcnum = 2;
  }
#endif
#if defined(ADC3)
  else if (adc == ADC3)
  {
    DIAG(F("ADCee::init(): found pin %d on ADC3"), pin);
    adcnum = 3;
  }
#endif
  else DIAG(F("ADCee::init(): found pin %d on unknown ADC!"), pin);

    // Port config - find which port we're on and power it up
    GPIO_TypeDef *gpioBase;

    switch (stmgpio)
    {
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
    case 0x03:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //Power up PORTD
        gpioBase = GPIOD;
        break;
    case 0x04:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //Power up PORTE
        gpioBase = GPIOE;
        break;
#if defined(GPIOF)
    case 0x05:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN; //Power up PORTF
        gpioBase = GPIOF;
        break;
#endif
    default:
      return -1023; // some silly value as error
  }

  // Set pin mux mode to analog input, the 32 bit port mode register has 2 bits per pin
  gpioBase->MODER |= (0b011 << (STM_PIN(stmpin) << 1)); // Set pin mux to analog mode (binary 11)

  // Set the sampling rate for that analog input
  // This is F411x specific! Different on for example F334
  // STM32F11xC/E Reference manual
  // 11.12.4 ADC sample time register 1 (ADC_SMPR1) (channels 10 to 18)
  // 11.12.5 ADC sample time register 2 (ADC_SMPR2) (channels 0 to 9)
  if (adcchan > 18)
    return -1022; // silly value as error
  if (adcchan < 10)
    adc->SMPR2 |= (0b111 << (adcchan * 3)); // Channel sampling rate 480 cycles
  else
    adc->SMPR1 |= (0b111 << ((adcchan - 10) * 3)); // Channel sampling rate 480 cycles

  // Read the inital ADC value for this analog input
  adc->SQR3 = adcchan;           // 1st conversion in regular sequence
  adc->CR2 |= ADC_CR2_SWSTART;   //(1 << 30);                     // Start 1st conversion SWSTART
  while(!(adc->SR & (1 << 1)));  // Wait until conversion is complete
  value = adc->DR;               // Read value from register

  uint8_t id = pin - PNUM_ANALOG_BASE;
  // if (id > 15) { // today we have not enough bits in the mask to support more
  //   return -1021;
  // }

  if (analogvals == NULL) {  // allocate analogvals, analogchans and adcchans if this is the first invocation of init
    analogvals = (int *)calloc(NUM_ADC_INPUTS+1, sizeof(int));
    analogchans = (uint32_t *)calloc(NUM_ADC_INPUTS+1, sizeof(uint32_t));
    adcchans = (ADC_TypeDef **)calloc(NUM_ADC_INPUTS+1, sizeof(ADC_TypeDef));
  }
  analogvals[id] = value;     // Store sampled value
  analogchans[id] = adcchan;  // Keep track of which ADC channel is used for reading this pin
  adcchans[id] = adc;         // Keep track of which ADC this channel is on
  usedpins |= (1 << id);                // This pin is now ready
  if (id > highestPin) highestPin = id; // Store our highest pin in use

  DIAG(F("ADCee::init(): value=%d, ADC%d: channel=%d, id=%d"), value, adcnum, adcchan, id);

  return value;
}

/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  uint8_t id = pin - PNUM_ANALOG_BASE;
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
  static uint8_t id = 0;     // id and mask are the same thing but it is faster to
  static uint16_t mask = 1;  // increment and shift instead to calculate mask from id
  static bool waiting = false;
  static ADC_TypeDef *adc;

  adc = adcchans[id];
  if (waiting)
  {
    // look if we have a result
    if (!(adc->SR & (1 << 1)))
      return; // no result, continue to wait
    // found value
    analogvals[id] = adc->DR;
    // advance at least one track
#ifdef DEBUG_ADC
    if (id == 1) TrackManager::track[1]->setBrake(0);
#endif
    waiting = false;
    id++;
    mask = mask << 1;
    if (id > highestPin) { // the 1 has been shifted out
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
        adc = adcchans[id];
        adc->SQR3 = analogchans[id]; // 1st conversion in regular sequence
        adc->CR2 |= (1 << 30);       // Start 1st conversion SWSTART
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
  //ADC1 config sequence
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
  // Set ADC prescaler - DIV8 ~ 40ms, DIV6 ~ 30ms, DIV4 ~ 20ms, DIV2 ~ 11ms
  ADC->CCR = (0 << 16); // Set prescaler 0=DIV2, 1=DIV4, 2=DIV6, 3=DIV8
  ADC1->CR1 &= ~(1 << 8); //SCAN mode disabled (Bit8)
  ADC1->CR1 &= ~(3 << 24); //12bit resolution (Bit24,25 0b00)
  ADC1->SQR1 = (1 << 20); //Set number of conversions projected (L[3:0] 0b0001) -> 1 conversion
  // Disable the DMA controller for ADC1
  ADC1->CR2 &= ~ADC_CR2_DMA;
  ADC1->CR2 &= ~(1 << 1); //Single conversion
  ADC1->CR2 &= ~(1 << 11); //Right alignment of data bits bit12....bit0
  ADC1->SQR1 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->SQR2 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->SQR3 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC1->CR2 |= (1 << 0); // Switch on ADC1
  // Wait for ADC1 to become ready (calibration complete)
  while (!(ADC1->CR2 & ADC_CR2_ADON)) {
  }
#if defined(ADC2)
  // Enable the ADC2 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

  // Initialize ADC2
  ADC2->CR1 = 0; // Disable all channels
  ADC2->CR2 = 0; // Clear CR2 register

  ADC2->CR1 &= ~(1 << 8); //SCAN mode disabled (Bit8)
  ADC2->CR1 &= ~(3 << 24); //12bit resolution (Bit24,25 0b00)
  ADC2->SQR1 = (1 << 20); //Set number of conversions projected (L[3:0] 0b0001) -> 1 conversion
  ADC2->CR2 &= ~ADC_CR2_DMA;   // Disable the DMA controller for ADC3
  ADC2->CR2 &= ~(1 << 1); //Single conversion
  ADC2->CR2 &= ~(1 << 11); //Right alignment of data bits bit12....bit0
  ADC2->SQR1 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC2->SQR2 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC2->SQR3 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register

  // Enable the ADC
  ADC2->CR2 |= ADC_CR2_ADON;

  // Wait for ADC2 to become ready (calibration complete)
  while (!(ADC2->CR2 & ADC_CR2_ADON)) {
  }

  // Perform ADC3 calibration (optional)
  // ADC3->CR2 |= ADC_CR2_CAL;
  // while (ADC3->CR2 & ADC_CR2_CAL) {
  // }
#endif
#if defined(ADC3)
  // Enable the ADC3 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

  // Initialize ADC3
  ADC3->CR1 = 0; // Disable all channels
  ADC3->CR2 = 0; // Clear CR2 register

  ADC3->CR1 &= ~(1 << 8); //SCAN mode disabled (Bit8)
  ADC3->CR1 &= ~(3 << 24); //12bit resolution (Bit24,25 0b00)
  ADC3->SQR1 = (1 << 20); //Set number of conversions projected (L[3:0] 0b0001) -> 1 conversion
  ADC3->CR2 &= ~ADC_CR2_DMA;   // Disable the DMA controller for ADC3
  ADC3->CR2 &= ~(1 << 1); //Single conversion
  ADC3->CR2 &= ~(1 << 11); //Right alignment of data bits bit12....bit0
  ADC3->SQR1 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC3->SQR2 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register
  ADC3->SQR3 &= ~(0x3FFFFFFF); //Clear whole 1st 30bits in register

  // Enable the ADC
  ADC3->CR2 |= ADC_CR2_ADON;

  // Wait for ADC3 to become ready (calibration complete)
  while (!(ADC3->CR2 & ADC_CR2_ADON)) {
  }

  // Perform ADC3 calibration (optional)
  // ADC3->CR2 |= ADC_CR2_CAL;
  // while (ADC3->CR2 & ADC_CR2_CAL) {
  // }
#endif
  interrupts();
}
#endif
