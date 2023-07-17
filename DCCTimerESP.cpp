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

// ATTENTION: this file only compiles on an ESP8266 and ESP32
// On ESP32 we do not even use the functions but they are here for completeness sake
// Please refer to DCCTimer.h for general comments about how this class works
// This is to avoid repetition and duplication.

#ifdef ARDUINO_ARCH_ESP8266

#include "DCCTimer.h"
INTERRUPT_CALLBACK interruptHandler=0;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler=callback;
  timer1_disable();

  // There seem to be differnt ways to attach interrupt handler
  //    ETS_FRC_TIMER1_INTR_ATTACH(NULL, NULL);
  //    ETS_FRC_TIMER1_NMI_INTR_ATTACH(interruptHandler);
  // Let us choose the one from the API
  timer1_attachInterrupt(interruptHandler);

  // not exactly sure of order:
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(CLOCK_CYCLES);
}
// We do not support to use PWM to make the Waveform on ESP
bool IRAM_ATTR DCCTimer::isPWMPin(byte pin) {
  return false;
}
void IRAM_ATTR DCCTimer::setPWM(byte pin, bool high) {
}
void IRAM_ATTR DCCTimer::clearPWM() {
}

// Fake this as it should not be used
void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
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
  return ESP.getFreeHeap();
}
#endif

////////////////////////////////////////////////////////////////////////

#ifdef ARDUINO_ARCH_ESP32
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#undef ADC_INPUT_MAX_VALUE
#define ADC_INPUT_MAX_VALUE 4095 // 12 bit ADC
#define pinToADC1Channel(X) (adc1_channel_t)(((X) > 35) ? (X)-36 : (X)-28)

int IRAM_ATTR local_adc1_get_raw(int channel) {
  uint16_t adc_value;
  SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
  while (SENS.sar_slave_addr1.meas_status != 0);
  SENS.sar_meas_start1.meas1_start_sar = 0;
  SENS.sar_meas_start1.meas1_start_sar = 1;
  while (SENS.sar_meas_start1.meas1_done_sar == 0);
  adc_value = SENS.sar_meas_start1.meas1_data_sar;
  return adc_value;
}

#include "DCCTimer.h"
INTERRUPT_CALLBACK interruptHandler=0;

// https://www.visualmicro.com/page/Timer-Interrupts-Explained.aspx

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  // This should not be called on ESP32 so disable it
  return;
  interruptHandler = callback;
  hw_timer_t *timer = NULL;
  timer = timerBegin(0, 2, true); // prescaler can be 2 to 65536 so choose 2
  timerAttachInterrupt(timer, interruptHandler, true);
  timerAlarmWrite(timer, CLOCK_CYCLES / 6, true); // divide by prescaler*3 (Clockbase is 80Mhz and not F_CPU 240Mhz)
  timerAlarmEnable(timer);
}

// We do not support to use PWM to make the Waveform on ESP
bool IRAM_ATTR DCCTimer::isPWMPin(byte pin) {
  return false;
}
void IRAM_ATTR DCCTimer::setPWM(byte pin, bool high) {
}
void IRAM_ATTR DCCTimer::clearPWM() {
}

// Fake this as it should not be used
void   DCCTimer::getSimulatedMacAddress(byte mac[6]) {
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
  return ESP.getFreeHeap();
}

void DCCTimer::reset() {
   ESP.restart();
}

#include "esp32-hal.h"
#include "soc/soc_caps.h"


#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif

static int8_t pin_to_channel[SOC_GPIO_PIN_COUNT] = { 0 };
static int cnt_channel = LEDC_CHANNELS;

void DCCTimer::DCCEXanalogWriteFrequency(uint8_t pin, uint32_t frequency) {
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] != 0) {
      ledcSetup(pin_to_channel[pin], frequency, 8);
    }
  }
}

void DCCTimer::DCCEXanalogWrite(uint8_t pin, int value) {
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] == 0) {
      if (!cnt_channel) {
          log_e("No more PWM channels available! All %u already used", LEDC_CHANNELS);
          return;
      }
      pin_to_channel[pin] = --cnt_channel;
      ledcSetup(cnt_channel, 1000, 8);
      ledcAttachPin(pin, cnt_channel);
    } else {
      ledcAttachPin(pin, pin_to_channel[pin]);
    }
    ledcWrite(pin_to_channel[pin], value);
  }
}

int ADCee::init(uint8_t pin) {
  pinMode(pin, ANALOG);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(pinToADC1Channel(pin),ADC_ATTEN_DB_11);
  return adc1_get_raw(pinToADC1Channel(pin));
}
int16_t ADCee::ADCmax() {
  return 4095;
}
/*
 * Read function ADCee::read(pin) to get value instead of analogRead(pin)
 */
int ADCee::read(uint8_t pin, bool fromISR) {
  return local_adc1_get_raw(pinToADC1Channel(pin));
}
/*
 * Scan function that is called from interrupt
 */
void ADCee::scan() {
}

void ADCee::begin() {
}

#endif //ESP32

