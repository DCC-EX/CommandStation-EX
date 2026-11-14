/*
 *  © 2026 Mathew Winters (with Github copilot)
 * 
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

#ifndef IO_EX_NeoPixelRMT_H
#define IO_EX_NeoPixelRMT_H

#if defined(ARDUINO_ARCH_ESP32)

#include "IO_NeoPixel.h"
#include <esp_idf_version.h>
#include <esp_heap_caps.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0) && !defined(MOTOR_SHIELD_TYPE)
#define NEOPIXEL_RMT_USE_IDF5
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#include <soc/soc_caps.h>
#else
#include <driver/rmt.h>
#endif
#include <freertos/FreeRTOS.h>

#define MAX_LEDS 256

class NeoPixelRMT : public IODevice {
public:
  static void create(VPIN vpin, int nPixels, int dataPin,
                     uint16_t mode=(NEO_GRB | NEO_KHZ800)) {
    if (nPixels > 0 && static_cast<uint32_t>(vpin) + nPixels <= 0x10000UL &&
        checkNoOverlapRange(vpin, nPixels))
      new NeoPixelRMT(vpin, nPixels, dataPin, mode);
  }

private:
#ifdef NEOPIXEL_RMT_USE_IDF5
  using RMTItem = rmt_symbol_word_t;
#else
  using RMTItem = rmt_item32_t;
#endif

  struct RGB {
    byte red;
    byte green;
    byte blue;
  };

  // 40 MHz ticks: match Adafruit's Arduino 3 ESP32 waveform.
  static constexpr uint16_t TICKS_800_T0H = 16;
  static constexpr uint16_t TICKS_800_T0L = 32;
  static constexpr uint16_t TICKS_800_T1H = 32;
  static constexpr uint16_t TICKS_800_T1L = 16;
  static constexpr uint16_t TICKS_400_T0H = 35;
  static constexpr uint16_t TICKS_400_T0L = 65;
  static constexpr uint16_t TICKS_400_T1H = 70;
  static constexpr uint16_t TICKS_400_T1L = 30;
  static constexpr uint16_t RESET_TICKS = 12000;

  static bool checkNoOverlapRange(VPIN firstVpin, int nPixels) {
    while (nPixels > 0) {
      uint8_t chunk = nPixels > UINT8_MAX ? UINT8_MAX : nPixels;
      if (!checkNoOverlap(firstVpin, chunk)) return false;
      firstVpin += chunk;
      nPixels -= chunk;
    }
    return true;
  }

  NeoPixelRMT(VPIN firstVpin, int nPixels, int dataPin, uint16_t mode)
      : IODevice(firstVpin, nPixels), _dataPin(dataPin) {
    _deviceState = DEVSTATE_INITIALISING;
    _redOffset = (mode >> 4) & 0x03;
    _greenOffset = (mode >> 2) & 0x03;
    _blueOffset = mode & 0x03;
    _whiteOffset = (mode >> 6) & 0x03;
    _bytesPerPixel = _whiteOffset == _redOffset ? 3 : 4;
    _is800KHz = (mode & NEO_KHZ400) == 0;

    if (nPixels <= 0 || static_cast<size_t>(nPixels) > MAX_LEDS) {
      DIAG(F("NeoPixel RMT number of leds %d > max of %d"), nPixels, MAX_LEDS);
      _deviceState = DEVSTATE_FAILED;
    } else {
      _pixels = static_cast<RGB *>(malloc(nPixels * sizeof(RGB)));
      _on = static_cast<byte *>(calloc(nPixels, sizeof(byte)));
      _frame = static_cast<RGB *>(malloc(nPixels * sizeof(RGB)));
      _frameOn = static_cast<byte *>(malloc(nPixels * sizeof(byte)));
      _items = static_cast<RMTItem *>(heap_caps_malloc(
          (static_cast<size_t>(nPixels) * _bytesPerPixel * 8 + 1) * sizeof(RMTItem),
          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

      if (_pixels == nullptr || _on == nullptr || _frame == nullptr ||
          _frameOn == nullptr || _items == nullptr) {
        free(_pixels);
        free(_on);
        free(_frame);
        free(_frameOn);
        free(_items);
        _pixels = nullptr;
        _on = nullptr;
        _frame = nullptr;
        _frameOn = nullptr;
        _items = nullptr;
        DIAG(F("NeoPixel RMT Out of Memory"));
        _deviceState = DEVSTATE_FAILED;
      } else {
        memset(_pixels, 0xFF, nPixels * sizeof(RGB));
#ifndef NEOPIXEL_RMT_USE_IDF5
        _channel = allocateChannel();
        if (_channel == RMT_CHANNEL_MAX) {
          DIAG(F("NeoPixel RMT Out of Memory"));
          _deviceState = DEVSTATE_FAILED;
        }
#endif
      }
    }

    _nextNeoPixel = devices();
    devices() = this;
    addDevice(this);
  }

#ifndef NEOPIXEL_RMT_USE_IDF5
  static rmt_channel_t allocateChannel() {
#ifdef MOTOR_SHIELD_TYPE
    static uint8_t nextChannel = 4;
#else
    static uint8_t nextChannel = 0;
#endif
    if (nextChannel >= RMT_CHANNEL_MAX) {
      DIAG(F("NeoPixel RMT Could not allocate RMT channel"));
      return RMT_CHANNEL_MAX;
    }
    return static_cast<rmt_channel_t>(nextChannel++);
  }
#endif

  void _begin() override {
    if (_rmtReady) return;
    if (_deviceState == DEVSTATE_FAILED) {
      _display();
      return;
    }

    _deviceState = DEVSTATE_INITIALISING;
#ifdef NEOPIXEL_RMT_USE_IDF5
    rmt_tx_channel_config_t config = {};
    config.gpio_num = static_cast<gpio_num_t>(_dataPin);
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = 40000000;
    // Two blocks double the refill deadline without monopolising the peripheral.
    config.mem_block_symbols = 2 * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    config.trans_queue_depth = 1;
    config.intr_priority = 3;
    auto _rmtError = rmt_new_tx_channel(&config, &_txChannel);
    rmt_copy_encoder_config_t encoderConfig = {};
    if (_rmtError == ESP_OK)
      _rmtError = rmt_new_copy_encoder(&encoderConfig, &_encoder);
    if (_rmtError == ESP_OK)
      _rmtError = rmt_enable(_txChannel);
#else
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
        static_cast<gpio_num_t>(_dataPin), _channel);
    config.clk_div = 2;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    auto _rmtError = rmt_config(&config);
    if (_rmtError == ESP_OK) {
      _rmtError = rmt_driver_install(_channel, 0, 0);
    }
#endif
    if (_rmtError != ESP_OK) {
#ifdef NEOPIXEL_RMT_USE_IDF5
      if (_encoder != nullptr) {
        esp_err_t cleanupError = rmt_del_encoder(_encoder);
        if (cleanupError != ESP_OK)
          DIAG(F("NeoPixel RMT encoder cleanup error:%d"), cleanupError);
        _encoder = nullptr;
      }
      if (_txChannel != nullptr) {
        esp_err_t cleanupError = rmt_del_channel(_txChannel);
        if (cleanupError != ESP_OK)
          DIAG(F("NeoPixel RMT channel cleanup error:%d"), cleanupError);
        _txChannel = nullptr;
      }
#endif
      failRmt(_rmtError);
      return;
    }

    _rmtReady = true;
  }

  void failRmt(esp_err_t error) {
    DIAG(F("NeoPixel RMT GPIO:%d error:%d"), _dataPin, error);
    _rmtFailed = true;
    _rmtReady = false;
    _deviceState = DEVSTATE_FAILED;
    _display();
  }

  bool copyFrame() {
    bool framePending;
    portENTER_CRITICAL(&_bufferMux);
    framePending = _framePending;
    if (framePending) {
      memcpy(_frame, _pixels, _nPins * sizeof(RGB));
      memcpy(_frameOn, _on, _nPins * sizeof(byte));
      _framePending = false;
    }
    portEXIT_CRITICAL(&_bufferMux);
    return framePending;
  }

  void encodeFrame() {
    const uint16_t t0h = _is800KHz ? TICKS_800_T0H : TICKS_400_T0H;
    const uint16_t t0l = _is800KHz ? TICKS_800_T0L : TICKS_400_T0L;
    const uint16_t t1h = _is800KHz ? TICKS_800_T1H : TICKS_400_T1H;
    const uint16_t t1l = _is800KHz ? TICKS_800_T1L : TICKS_400_T1L;
    size_t item = 0;

    for (int pixel = 0; pixel < _nPins; pixel++) {
      byte channels[4] = {0, 0, 0, 0};
      if (_frameOn[pixel]) {
        RGB colour = _frame[pixel];
        if (_bytesPerPixel == 4 && colour.red == colour.green &&
            colour.red == colour.blue) {
          channels[_whiteOffset] = colour.red;
        } else {
          channels[_redOffset] = colour.red;
          channels[_greenOffset] = colour.green;
          channels[_blueOffset] = colour.blue;
        }
      }

      for (byte channel = 0; channel < _bytesPerPixel; channel++) {
        for (int bit = 7; bit >= 0; bit--) {
          bool one = channels[channel] & (1 << bit);
          _items[item].level0 = 1;
          _items[item].duration0 = one ? t1h : t0h;
          _items[item].level1 = 0;
          _items[item].duration1 = one ? t1l : t0l;
          item++;
        }
      }
    }

    _items[item].level0 = 0;
    _items[item].duration0 = RESET_TICKS / 2;
    _items[item].level1 = 0;
    _items[item].duration1 = RESET_TICKS / 2;
    _itemCount = item + 1;
  }

  void _loop(unsigned long) override {
    if (_rmtFailed && _deviceState != DEVSTATE_FAILED) {
      _deviceState = DEVSTATE_FAILED;
      _display();
    } else if (_rmtReady && _deviceState != DEVSTATE_NORMAL) {
      _deviceState = DEVSTATE_NORMAL;
      _display();
    }

    if (!_rmtReady || _rmtFailed) return;

    if (_transmitting) {
#ifdef NEOPIXEL_RMT_USE_IDF5
      auto _rmtError = rmt_tx_wait_all_done(_txChannel, 0);
#else
      auto _rmtError = rmt_wait_tx_done(_channel, 0);
#endif
      if (_rmtError == ESP_ERR_TIMEOUT) return;
      if (_rmtError != ESP_OK) {
        failRmt(_rmtError);
        return;
      }
      _transmitting = false;
    }

    if (!copyFrame()) return;
    encodeFrame();
#ifdef NEOPIXEL_RMT_USE_IDF5
    rmt_transmit_config_t transmitConfig = {};
    auto _rmtError = rmt_transmit(_txChannel, _encoder, _items,
                                _itemCount * sizeof(RMTItem), &transmitConfig);
#else
    auto _rmtError = rmt_write_items(_channel, _items, _itemCount, false);
#endif
    if (_rmtError != ESP_OK) {
      failRmt(_rmtError);
      return;
    }
    _transmitting = true;
  }

  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED || _on == nullptr) return 0;
    size_t pixel = vpin - _firstVpin;
    portENTER_CRITICAL(&_bufferMux);
    int value = _on[pixel];
    portEXIT_CRITICAL(&_bufferMux);
    return value;
  }

  void _write(VPIN vpin, int value) override {
    if (_deviceState == DEVSTATE_FAILED || _on == nullptr) return;
    size_t pixel = vpin - _firstVpin;
    byte on = value ? 1 : 0;
    portENTER_CRITICAL(&_bufferMux);
    bool changed = _on[pixel] != on;
    _on[pixel] = on;
    if (changed) _framePending = true;
    portEXIT_CRITICAL(&_bufferMux);
  }

  void _writeAnalogue(VPIN vpin, int colourRG, uint8_t onoff,
                      uint16_t colourB) override {
    if (_deviceState == DEVSTATE_FAILED || _pixels == nullptr || _on == nullptr) return;
    size_t pixel = vpin - _firstVpin;
    RGB colour = {static_cast<byte>((colourRG >> 8) & 0xFF),
                  static_cast<byte>(colourRG & 0xFF),
                  static_cast<byte>(colourB & 0xFF)};
    portENTER_CRITICAL(&_bufferMux);
    bool changed = _pixels[pixel].red != colour.red ||
                   _pixels[pixel].green != colour.green ||
                   _pixels[pixel].blue != colour.blue ||
                   _on[pixel] != (onoff ? 1 : 0);
    _pixels[pixel] = colour;
    _on[pixel] = onoff ? 1 : 0;
    if (changed) _framePending = true;
    portEXIT_CRITICAL(&_bufferMux);
  }

  void _display() override {
    DIAG(F("NeoPixel RMT GPIO:%d Vpins %u-%u %S"), _dataPin,
         (int)_firstVpin, (int)_firstVpin + _nPins - 1,
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F("ONLINE"));
  }

  static NeoPixelRMT *&devices() {
    static NeoPixelRMT *firstDevice = nullptr;
    return firstDevice;
  }

  NeoPixelRMT *_nextNeoPixel = nullptr;
  const int _dataPin;
#ifdef NEOPIXEL_RMT_USE_IDF5
  rmt_channel_handle_t _txChannel = nullptr;
  rmt_encoder_handle_t _encoder = nullptr;
#else
  rmt_channel_t _channel = RMT_CHANNEL_MAX;
#endif
  RGB *_pixels = nullptr;
  byte *_on = nullptr;
  RGB *_frame = nullptr;
  byte *_frameOn = nullptr;
  RMTItem *_items = nullptr;
  size_t _itemCount = 0;
  byte _bytesPerPixel = 3;
  byte _redOffset = 1;
  byte _greenOffset = 0;
  byte _blueOffset = 2;
  byte _whiteOffset = 1;
  bool _is800KHz = true;
  bool _framePending = true;
  bool _transmitting = false;
  volatile bool _rmtReady = false;
  volatile bool _rmtFailed = false;
  portMUX_TYPE _bufferMux = portMUX_INITIALIZER_UNLOCKED;
};

#undef NEOPIXEL_RMT_USE_IDF5

#endif // ARDUINO_ARCH_ESP32

#endif // IO_EX_NeoPixelRMT_H