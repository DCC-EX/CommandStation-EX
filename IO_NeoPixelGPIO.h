/*
 *  © 2024, Chris Harlow. All rights reserved.
 *
 *  This file is part of EX-CommandStation
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

// ESP32 direct GPIO NeoPixels. See Release_Notes/NeoPixelGPIO.md.
#ifndef IO_NeoPixelGPIO_H
#define IO_NeoPixelGPIO_H

#if defined(ARDUINO_ARCH_ESP32)
#include "IODevice.h"
#include "defines.h"
#include "DIAG.h"
#include <Adafruit_NeoPixel.h>
#include <driver/gpio.h>
#include <esp_idf_version.h>
#if ESP_IDF_VERSION_MAJOR < 5 && defined(MOTOR_SHIELD_TYPE)
#include <driver/rmt.h>
// Adafruit NeoPixel 1.15.5's legacy ESP32 backend consults this table.
// DCCRMT owns channels 0-3, including the extra memory blocks.
extern bool rmt_reserved_channels[];
#endif

class NeoPixelGPIO : public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, int16_t gpioPin,
                     uint16_t mode = NEO_GRB | NEO_KHZ800) {
    const int bytesPerPixel = ((mode >> 6 & 3) == (mode >> 4 & 3)) ? 3 : 4;
    if (nPins <= 0 || nPins > 65535 / bytesPerPixel ||
        firstVpin > VPIN_MAX || nPins > VPIN_MAX - firstVpin + 1 ||
        !GPIO_IS_VALID_OUTPUT_GPIO(gpioPin)) {
      DIAG(F("NeoPixelGPIO invalid VPIN range, length or GPIO:%d"), gpioPin);
      return;
    }
    // checkNoOverlap takes an 8-bit count; check long strings in chunks.
    for (int offset = 0; offset < nPins; offset += 255) {
      int count = nPins - offset;
      if (!checkNoOverlap(firstVpin + offset, count > 255 ? 255 : count)) return;
    }
    new NeoPixelGPIO(firstVpin, nPins, gpioPin, mode);
  }

private:
  struct RGB { uint8_t red, green, blue; };
  Adafruit_NeoPixel _strip;
  RGB *_colours = nullptr;
  uint8_t *_states = nullptr;
  bool _showPending = false;
  bool _rgbw;

  NeoPixelGPIO(VPIN firstVpin, int nPins, int16_t gpioPin, uint16_t mode)
      : IODevice(firstVpin, nPins), _strip(nPins, gpioPin, mode),
        _rgbw((mode >> 6 & 3) != (mode >> 4 & 3)) {
    _colours = (RGB *)malloc(nPins * sizeof(RGB));
    _states = (uint8_t *)calloc((nPins + 7) / 8, 1);
    if (!_colours || !_states || !_strip.getPixels()) {
      free(_colours);
      free(_states);
      _colours = nullptr;
      _states = nullptr;
      _deviceState = DEVSTATE_FAILED;
      DIAG(F("NeoPixelGPIO GPIO:%d not enough RAM"), gpioPin);
    } else {
      memset(_colours, 255, nPins * sizeof(RGB));
    }
    addDevice(this);
  }

  void _begin() override {
    if (!_colours || !_states || !_strip.getPixels()) {
      _deviceState = DEVSTATE_FAILED;
      return;
    }
#if ESP_IDF_VERSION_MAJOR < 5 && defined(MOTOR_SHIELD_TYPE)
    if (RMT_CHANNEL_MAX <= 4) {
      DIAG(F("NeoPixelGPIO no spare RMT channel alongside DCC"));
      _deviceState = DEVSTATE_FAILED;
      return;
    }
    for (int channel = 0; channel < 4; channel++)
      rmt_reserved_channels[channel] = true;
#endif
    if (!_strip.begin()) {
      _deviceState = DEVSTATE_FAILED;
      _display();
      return;
    }
    // Rebuild the output on HAL restart, retaining colours and on/off state.
    for (int pixel = 0; pixel < _nPins; pixel++) transmit(pixel);
    _display();
  }

  void _loop(unsigned long currentMicros) override {
    (void)currentMicros;
    if (_deviceState == DEVSTATE_FAILED || !_showPending || !_strip.canShow()) return;
    _strip.show();
    _showPending = false;
  }

  bool isOn(int pixel) const { return _states[pixel / 8] & (1 << (pixel % 8)); }
  void setOn(int pixel, bool on) {
    if (on) _states[pixel / 8] |= 1 << (pixel % 8);
    else _states[pixel / 8] &= ~(1 << (pixel % 8));
  }

  int _read(VPIN vpin) override {
    return _deviceState != DEVSTATE_FAILED && isOn(vpin - _firstVpin);
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pixel = vpin - _firstVpin;
    if (isOn(pixel) == bool(value)) return;
    setOn(pixel, value);
    transmit(pixel);
  }

  void _writeAnalogue(VPIN vpin, int colourRG, uint8_t onoff, uint16_t colourB) override {
    if (_deviceState == DEVSTATE_FAILED) return;
    int pixel = vpin - _firstVpin;
    RGB colour = {uint8_t(colourRG >> 8), uint8_t(colourRG), uint8_t(colourB)};
    RGB &old = _colours[pixel];
    if (old.red == colour.red && old.green == colour.green &&
        old.blue == colour.blue && isOn(pixel) == bool(onoff)) return;
    old = colour;
    setOn(pixel, onoff);
    transmit(pixel);
  }

  VPIN rangeEnd(VPIN vpin, int count) const {
    int remaining = _firstVpin + _nPins - vpin;
    return vpin + (count <= 0 ? 0 : (count < remaining ? count : remaining));
  }

  VPIN _writeRange(VPIN vpin, int value, int count) override {
    VPIN end = rangeEnd(vpin, count);
    while (vpin < end) _write(vpin++, value);
    return end;
  }

  VPIN _writeAnalogueRange(VPIN vpin, int rg, uint8_t onoff, uint16_t b, int count) override {
    VPIN end = rangeEnd(vpin, count);
    while (vpin < end) _writeAnalogue(vpin++, rg, onoff, b);
    return end;
  }

  void transmit(int pixel) {
    RGB colour = isOn(pixel) ? _colours[pixel] : RGB{0, 0, 0};
    if (_rgbw && colour.red == colour.green && colour.red == colour.blue)
      _strip.setPixelColor(pixel, 0, 0, 0, colour.red);
    else
      _strip.setPixelColor(pixel, colour.red, colour.green, colour.blue, 0);
    _showPending = true;
  }

  void _display() override {
    DIAG(F("NeoPixelGPIO GPIO:%d Vpins %u-%u %S"), _strip.getPin(),
         (int)_firstVpin, (int)(_firstVpin + _nPins - 1),
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
};
#endif // ARDUINO_ARCH_ESP32
#endif
