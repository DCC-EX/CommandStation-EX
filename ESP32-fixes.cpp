/*
 *  © 2022 Harald Barth
 *  All rights reserved.
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
#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include "ESP32-fixes.h"

#include "esp32-hal.h"
#include "soc/soc_caps.h"


#ifdef SOC_LEDC_SUPPORT_HS_MODE
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM<<1)
#else
#define LEDC_CHANNELS           (SOC_LEDC_CHANNEL_NUM)
#endif

static int8_t pin_to_channel[SOC_GPIO_PIN_COUNT] = { 0 };
static int cnt_channel = LEDC_CHANNELS;

void DCCEXanalogWriteFrequency(uint8_t pin, uint32_t frequency) {
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] != 0) {
      ledcSetup(pin_to_channel[pin], frequency, 8);
    }
  }
}

void DCCEXanalogWrite(uint8_t pin, int value) {
  if (pin < SOC_GPIO_PIN_COUNT) {
    if (pin_to_channel[pin] == 0) {
      if (!cnt_channel) {
          log_e("No more PWM channels available! All %u already used", LEDC_CHANNELS);
          return;
      }
      pin_to_channel[pin] = --cnt_channel;
      ledcAttachPin(pin, cnt_channel);
      ledcSetup(cnt_channel, 1000, 8);
    } else {
      ledcAttachPin(pin, pin_to_channel[pin]);
    }
    ledcWrite(pin_to_channel[pin], value);
  }
}
#endif
