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
INTERRUPT_CALLBACK interruptHandler = 0;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  interruptHandler = callback;
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
void DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  mac[0] = 0xFE;
  mac[1] = 0xBE;
  mac[2] = 0xEF;
  mac[3] = 0xC0;
  mac[4] = 0xFF;
  mac[5] = 0xEE;
}

volatile int DCCTimer::minimum_free_memory = __INT_MAX__;

// Return low memory value...
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts();  // Disable interrupts to get volatile value
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

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif
#if ESP_IDF_VERSION_MAJOR == 4
// all well correct IDF version
#else
#error \
    "DCC-EX does not support compiling with IDF version 5.0 or later. Downgrade your ESP32 library to a version that contains IDF version 4. Arduino ESP32 library 3.0.0 is too new. Downgrade to one of 2.0.9 to 2.0.17"
#endif

// protect all the rest of the code from IDF version 5
#if ESP_IDF_VERSION_MAJOR == 4
#include "DIAG.h"
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#undef ADC_INPUT_MAX_VALUE
#define ADC_INPUT_MAX_VALUE 4095  // 12 bit ADC
#define pinToADC1Channel(X) (adc1_channel_t)(((X) > 35) ? (X) - 36 : (X) - 28)

int IRAM_ATTR local_adc1_get_raw(int channel) {
  uint16_t adc_value;
  SENS.sar_meas_start1.sar1_en_pad = (1 << channel);  // only one channel is selected
  while (SENS.sar_slave_addr1.meas_status != 0);
  SENS.sar_meas_start1.meas1_start_sar = 0;
  SENS.sar_meas_start1.meas1_start_sar = 1;
  while (SENS.sar_meas_start1.meas1_done_sar == 0);
  adc_value = SENS.sar_meas_start1.meas1_data_sar;
  return adc_value;
}

#include "DCCTimer.h"
INTERRUPT_CALLBACK interruptHandler = 0;

// https://www.visualmicro.com/page/Timer-Interrupts-Explained.aspx

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void DCCTimer::begin(INTERRUPT_CALLBACK callback) {
  // This should not be called on ESP32 so disable it
  return;
  interruptHandler = callback;
  hw_timer_t* timer = NULL;
  timer = timerBegin(0, 2, true);  // prescaler can be 2 to 65536 so choose 2
  timerAttachInterrupt(timer, interruptHandler, true);
  timerAlarmWrite(timer, CLOCK_CYCLES / 6, true);  // divide by prescaler*3 (Clockbase is 80Mhz and not F_CPU 240Mhz)
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
void DCCTimer::getSimulatedMacAddress(byte mac[6]) {
  mac[0] = 0xFE;
  mac[1] = 0xBE;
  mac[2] = 0xEF;
  mac[3] = 0xC0;
  mac[4] = 0xFF;
  mac[5] = 0xEE;
}

volatile int DCCTimer::minimum_free_memory = __INT_MAX__;

// Return low memory value...
int DCCTimer::getMinimumFreeMemory() {
  noInterrupts();  // Disable interrupts to get volatile value
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

void DCCTimer::DCCEXanalogWriteFrequency(uint8_t pin, uint32_t f) {
  if (f >= 16)
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, f);
  /*
    else if (f == 7) // not used on ESP32
      DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 62500);
  */
  else if (f >= 4)
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 32000);
  else if (f >= 3)
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 16000);
  else if (f >= 2)
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 3400);
  else if (f == 1)
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 480);
  else
    DCCTimer::DCCEXanalogWriteFrequencyInternal(pin, 131);
}

#include "esp32-hal.h"
#include "soc/soc_caps.h"

#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS (SOC_LEDC_CHANNEL_NUM << 1)
#else
#define LEDC_CHANNELS (SOC_LEDC_CHANNEL_NUM)
#endif

static int8_t pin_to_channel[SOC_GPIO_PIN_COUNT] = {0};
static int cnt_channel = LEDC_CHANNELS;

void DCCTimer::DCCEXanalogWriteFrequencyInternal(uint8_t pin, uint32_t frequency) {
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] != 0) {
      ledcSetup(pin_to_channel[pin], frequency, 8);
    }
  }
}

void DCCTimer::DCCEXledcDetachPin(uint8_t pin) {
  DIAG(F("Clear pin %d channel"), pin);
  pin_to_channel[pin] = 0;
  pinMatrixOutDetach(pin, false, false);
}

static byte LEDCToMux[] = {
    LEDC_HS_SIG_OUT0_IDX, LEDC_HS_SIG_OUT1_IDX, LEDC_HS_SIG_OUT2_IDX, LEDC_HS_SIG_OUT3_IDX, LEDC_HS_SIG_OUT4_IDX, LEDC_HS_SIG_OUT5_IDX,
    LEDC_HS_SIG_OUT6_IDX, LEDC_HS_SIG_OUT7_IDX, LEDC_LS_SIG_OUT0_IDX, LEDC_LS_SIG_OUT1_IDX, LEDC_LS_SIG_OUT2_IDX, LEDC_LS_SIG_OUT3_IDX,
    LEDC_LS_SIG_OUT4_IDX, LEDC_LS_SIG_OUT5_IDX, LEDC_LS_SIG_OUT6_IDX, LEDC_LS_SIG_OUT7_IDX,
};

void DCCTimer::DCCEXledcAttachPin(uint8_t pin, int8_t channel, bool inverted) {
  DIAG(F("Attaching pin %d to channel %d %c"), pin, channel, inverted ? 'I' : ' ');
  ledcAttachPin(pin, channel);
  if (inverted)  // we attach again but with inversion
    gpio_matrix_out(pin, LEDCToMux[channel], inverted, 0);
}

void DCCTimer::DCCEXanalogCopyChannel(int8_t frompin, int8_t topin) {
  // arguments are signed depending on inversion of pins
  DIAG(F("Pin %d copied to %d"), frompin, topin);
  bool inverted = false;
  if (frompin < 0)
    frompin = -frompin;
  if (topin < 0) {
    inverted = true;
    topin = -topin;
  }
  int channel = pin_to_channel[frompin];  // after abs(frompin)
  pin_to_channel[topin] = channel;
  DCCTimer::DCCEXledcAttachPin(topin, channel, inverted);
}

void DCCTimer::DCCEXanalogWrite(uint8_t pin, int value, bool invert) {
  // This allocates channels 15, 13, 11, ....
  // so each channel gets its own timer.
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] == 0) {
      int search_channel;
      int n;
      if (!cnt_channel) {
        log_e("No more PWM channels available! All %u already used", LEDC_CHANNELS);
        return;
      }
      // search for free channels top down
      for (search_channel = LEDC_CHANNELS - 1; search_channel >= cnt_channel; search_channel -= 2) {
        bool chanused = false;
        for (n = 0; n < SOC_GPIO_PIN_COUNT; n++) {
          if (pin_to_channel[n] == search_channel) {  // current search_channel used
            chanused = true;
            break;
          }
        }
        if (chanused)
          continue;
        if (n == SOC_GPIO_PIN_COUNT)  // current search_channel unused
          break;
      }
      if (search_channel >= cnt_channel) {
        pin_to_channel[pin] = search_channel;
        DIAG(F("Pin %d assigned to search channel %d"), pin, search_channel);
      } else {
        pin_to_channel[pin] = --cnt_channel;  // This sets 15, 13, ...
        DIAG(F("Pin %d assigned to new channel %d"), pin, cnt_channel);
        --cnt_channel;  // Now we are at 14, 12, ...
      }
      ledcSetup(pin_to_channel[pin], 1000, 8);
      DCCEXledcAttachPin(pin, pin_to_channel[pin], invert);
    } else {
      // This else is only here so we can enable diag
      // Pin should be already attached to channel
      // DIAG(F("Pin %d assigned to old channel %d"), pin, pin_to_channel[pin]);
    }
    ledcWrite(pin_to_channel[pin], value);
  }
}

void DCCTimer::DCCEXInrushControlOn(uint8_t pin, int duty, bool inverted) {
  // this uses hardcoded channel 0
  ledcSetup(0, 62500, 8);
  DCCEXledcAttachPin(pin, 0, inverted);
  ledcWrite(0, duty);
}

int ADCee::init(uint8_t pin) {
  pinMode(pin, ANALOG);
  adc1_config_width(ADC_WIDTH_BIT_12);
// Espressif deprecated ADC_ATTEN_DB_11 somewhere between 2.0.9 and 2.0.17
#ifdef ADC_ATTEN_11db
  adc1_config_channel_atten(pinToADC1Channel(pin), ADC_ATTEN_11db);
#else
  adc1_config_channel_atten(pinToADC1Channel(pin), ADC_ATTEN_DB_11);
#endif
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
#endif  // IDF v4
#endif  // ESP32
