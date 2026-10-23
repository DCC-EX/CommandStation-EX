#ifndef IO_EX_NeoPixelRMT_H
#define IO_EX_NeoPixelRMT_H

#if defined(ARDUINO_ARCH_ESP32)

#include "IO_NeoPixel.h"
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class NeoPixelRMT : public IODevice {
public:
  static void create(VPIN vpin, int nPixels, int dataPin,
                     uint16_t mode=(NEO_GRB | NEO_KHZ800)) {
    if (nPixels > 0 && static_cast<uint32_t>(vpin) + nPixels <= 0x10000UL &&
        checkNoOverlapRange(vpin, nPixels))
      new NeoPixelRMT(vpin, nPixels, dataPin, mode);
  }

private:
  struct RGB {
    byte red;
    byte green;
    byte blue;
  };

  static constexpr uint16_t TICKS_800_T0H = 14;
  static constexpr uint16_t TICKS_800_T0L = 36;
  static constexpr uint16_t TICKS_800_T1H = 28;
  static constexpr uint16_t TICKS_800_T1L = 22;
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
    _redOffset = (mode >> 4) & 0x03;
    _greenOffset = (mode >> 2) & 0x03;
    _blueOffset = mode & 0x03;
    _whiteOffset = (mode >> 6) & 0x03;
    _bytesPerPixel = _whiteOffset == _redOffset ? 3 : 4;
    _is800KHz = (mode & NEO_KHZ400) == 0;

    if (nPixels <= 0 || static_cast<size_t>(nPixels) >
        (SIZE_MAX - 2) / (_bytesPerPixel * 8)) {
      _rmtError = ESP_ERR_INVALID_SIZE;
      addDevice(this);
      return;
    }

    _pixels = static_cast<RGB *>(malloc(nPixels * sizeof(RGB)));
    _on = static_cast<byte *>(calloc(nPixels, sizeof(byte)));
    _frame = static_cast<RGB *>(malloc(nPixels * sizeof(RGB)));
    _frameOn = static_cast<byte *>(malloc(nPixels * sizeof(byte)));
    _items = static_cast<rmt_item32_t *>(malloc(
        (static_cast<size_t>(nPixels) * _bytesPerPixel * 8 + 2) * sizeof(rmt_item32_t)));

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
      _rmtError = ESP_ERR_NO_MEM;
      addDevice(this);
      return;
    }

    memset(_pixels, 0xFF, nPixels * sizeof(RGB));
    _channel = allocateChannel();
    if (_channel == RMT_CHANNEL_MAX) _rmtError = ESP_ERR_INVALID_ARG;
    addDevice(this);
  }

  static rmt_channel_t allocateChannel() {
#ifdef MOTOR_SHIELD_TYPE
    static uint8_t nextChannel = 4;
#else
    static uint8_t nextChannel = 0;
#endif
    if (nextChannel >= RMT_CHANNEL_MAX) return RMT_CHANNEL_MAX;
    return static_cast<rmt_channel_t>(nextChannel++);
  }

  void _begin() override {
    if (_sender != nullptr) return;
    if (_rmtError != ESP_OK) {
      _deviceState = DEVSTATE_FAILED;
      _display();
      return;
    }

    _deviceState = DEVSTATE_INITIALISING;
    BaseType_t result = xTaskCreatePinnedToCore(
        senderTask, "NeoPixelRMT", 4096, this, 1, &_sender, 0);
    if (result != pdPASS) {
      _rmtError = ESP_ERR_NO_MEM;
      _deviceState = DEVSTATE_FAILED;
      _display();
    }
  }

  static void senderTask(void *arg) {
    static_cast<NeoPixelRMT *>(arg)->runSender();
  }

  void runSender() {
    _sender = xTaskGetCurrentTaskHandle();
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
        static_cast<gpio_num_t>(_dataPin), _channel);
    config.clk_div = 2;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    _rmtError = rmt_config(&config);
    if (_rmtError == ESP_OK) _rmtError = rmt_driver_install(_channel, 0, 0);
    if (_rmtError != ESP_OK) {
      _rmtFailed = true;
      vTaskDelete(nullptr);
      return;
    }

    _rmtReady = true;
    bool sendFrame = true;
    for (;;) {
      if (sendFrame) {
        copyFrame();
        encodeFrame();
        _rmtError = rmt_write_items(_channel, _items, _itemCount, true);
        if (_rmtError != ESP_OK) {
          _rmtFailed = true;
          _rmtReady = false;
          vTaskDelete(nullptr);
          return;
        }
      }
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      sendFrame = true;
    }
  }

  void copyFrame() {
    portENTER_CRITICAL(&_bufferMux);
    memcpy(_frame, _pixels, _nPins * sizeof(RGB));
    memcpy(_frameOn, _on, _nPins * sizeof(byte));
    portEXIT_CRITICAL(&_bufferMux);
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
    _items[item].duration0 = RESET_TICKS;
    _items[item].level1 = 0;
    _items[item].duration1 = 0;
    _itemCount = item + 1;
  }

  void _loop(unsigned long currentMicros) override {
    (void)currentMicros;
    if (_rmtFailed && _deviceState != DEVSTATE_FAILED) {
      _deviceState = DEVSTATE_FAILED;
      _display();
    } else if (_rmtReady && _deviceState != DEVSTATE_NORMAL) {
      _deviceState = DEVSTATE_NORMAL;
      _display();
    }
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
    portEXIT_CRITICAL(&_bufferMux);
    if (changed) notifySender();
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
    portEXIT_CRITICAL(&_bufferMux);
    if (changed) notifySender();
  }

  void notifySender() {
    if (_rmtReady && !_rmtFailed && _sender != nullptr)
      xTaskNotifyGive(_sender);
  }

  void _display() override {
    DIAG(F("NeoPixel RMT GPIO:%d Vpins %u-%u %S error:%d"), _dataPin,
         (int)_firstVpin, (int)_firstVpin + _nPins - 1,
         _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""),
         static_cast<int>(_rmtError));
  }

  const int _dataPin;
  rmt_channel_t _channel = RMT_CHANNEL_MAX;
  TaskHandle_t _sender = nullptr;
  RGB *_pixels = nullptr;
  byte *_on = nullptr;
  RGB *_frame = nullptr;
  byte *_frameOn = nullptr;
  rmt_item32_t *_items = nullptr;
  size_t _itemCount = 0;
  byte _bytesPerPixel = 3;
  byte _redOffset = 1;
  byte _greenOffset = 0;
  byte _blueOffset = 2;
  byte _whiteOffset = 1;
  bool _is800KHz = true;
  volatile bool _rmtReady = false;
  volatile bool _rmtFailed = false;
  volatile esp_err_t _rmtError = ESP_OK;
  portMUX_TYPE _bufferMux = portMUX_INITIALIZER_UNLOCKED;
};

#endif // ARDUINO_ARCH_ESP32

#endif // IO_EX_NeoPixelRMT_H