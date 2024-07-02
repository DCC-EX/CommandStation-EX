/*
 *  © 2020-2022 Harald Barth
 *
 *  This file is part of CommandStation-EX
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

////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_ARCH_RP2040

#include "pico/stdlib.h"

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/adc.h>
#include "hardware/timer.h"
#include "hardware/irq.h"

#if defined(ADC_INPUT_MAX_VALUE)
#undef ADC_INPUT_MAX_VALUE
#endif
#define ADC_INPUT_MAX_VALUE 4095 // 12 bit ADC
#define NUM_ADC_INPUTS  4

#include "DCCTimer.h"
INTERRUPT_CALLBACK interruptHandler=0;

#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

static uint32_t dcc_signal_time = 0;

static void alarm_irq(void) {
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    // Reload timer
    uint64_t target = timer_hw->timerawl + dcc_signal_time;
    timer_hw->alarm[ALARM_NUM] = (uint32_t) target;

    if (interruptHandler)
      interruptHandler();
}

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
    interruptHandler = callback;
    dcc_signal_time = DCC_SIGNAL_TIME;
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor

    // Alarm is only 32 bits so if trying to delay more
    // than that need to be careful and keep track of the upper
    // bits
    uint64_t target = timer_hw->timerawl + dcc_signal_time;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t) target;
}

// All pins are PWM capable
bool DCCTimer::isPWMPin(byte pin) {
  return false;
}

void DCCTimer::setPWM(byte pin, bool high) {
    (void) pin;
    (void) high;
}

void DCCTimer::clearPWM() {
}

// Fake this as it should not be used
void DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  mac[0] = 0xFE;
  mac[1] = 0xBE;
  mac[2] = 0xEF;
  mac[3] = 0xC0;
  mac[4] = 0xFF;
  mac[5] = 0xEE;
}

volatile int DCCTimer::minimum_free_memory=__INT_MAX__;

// Return low memory value... 
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts(); // Disable interrupts to get volatile value 
  int retval = minimum_free_memory;
  interrupts();
  return retval;
}

int DCCTimer::freeMemory() {
  return rp2040.getFreeHeap();
}

void DCCTimer::reset() {
   rp2040.reboot();
}

void DCCTimer::DCCEXanalogWriteFrequency(uint8_t pin, uint32_t frequency) {
    (void) pin; /* Can't set different frequencies on different pins */
    analogWriteFreq(frequency);
}

void DCCTimer::DCCEXanalogWrite(uint8_t pin, int value) {
    analogWrite(pin, value);
}

int ADCee::init(uint8_t pin) {
  uint8_t id = pin - A0;
  int value = 0;

  if (id > NUM_ADC_INPUTS)
    return -1023;

  adc_gpio_init(pin);
  adc_select_input(pin);

  return(adc_read());
}

int16_t ADCee::ADCmax() {
  return ADC_INPUT_MAX_VALUE;
}

/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  adc_select_input(pin);

  return(adc_read());
}

/*
 * Scan function that is called from interrupt
 */
void ADCee::scan() {
}

void ADCee::begin() {
    adc_init();
}

#endif // RP2040
