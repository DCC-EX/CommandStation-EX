# ESP32 GPIO NeoPixels

`IO_NeoPixelGPIO.h` drives a NeoPixel string directly from an ESP32 output GPIO
using Adafruit NeoPixel. The existing `NeoPixel` I2C NeoDriver remains available.

In `myAutomation.h`, name the physical GPIO and define the string:

```cpp
#define SIGNAL_LED_GPIO 25
HAL(NeoPixelGPIO, 1000, 20, SIGNAL_LED_GPIO, NEO_GRB | NEO_KHZ800)
```

Choose an available output GPIO for your board; 25 is only an example. The pin
is a physical GPIO number, not a HAL VPIN. Do not share it with a pin used for a
motor shield, I2C, serial, another ESP32 peripheral, or one of the internal boot
strapping or FLASH pins etc.

Pixels use VPINs, so in this instance it would use 20 pins starting at 1000, or 1000–1019.

The mode is optional and defaults to `NEO_GRB | NEO_KHZ800`.

For direct HAL setup, include `IO_NeoPixelGPIO.h` and call
`NeoPixelGPIO::create(1000, 20, SIGNAL_LED_GPIO, NEO_GRB);`.

Install **Adafruit NeoPixel 1.15.5** in Arduino Library Manager. The PlatformIO
ESP32 environments in `platformio.ini` declare this dependency.
For custom ESP32 environments add `adafruit/Adafruit NeoPixel @ 1.15.5` to
`lib_deps`. The driver is automatically included on ESP32; a missing library produces
an explicit include error. It is not available on other architectures.

The command API matches the NeoPixel I2C driver:

```cpp
NEOPIXEL(1000, 255, 0, 0)       // Red, on
NEOPIXEL(-1001, 0, 0, 255)      // Store blue, off
SET(1001)                       // On using stored blue
RESET(1001)                     // Off, retaining blue
NEOPIXEL(1005, 0, 255, 0, 5)    // Five green pixels
```

These are EXRAIL sequence instructions. `BLINK`, `NEOPIXEL_SIGNAL` and the
`<o>` commands work as for the I2C driver. Digital reads return on/off state.
Colours use all eight bits per component; the analogue write parameters are
packed red/green, on/off, and blue, as in `IO_NeoPixel.h`. No analogue colour
readback is provided. Pixels initially store white and are switched off.
RGBW modes use the white channel for equal red, green and blue values.

Updates are batched until the HAL loop runs. The library's `show()` transmits
the whole string synchronously, so long strings increase loop latency.

## ESP32 RMT resources

On Arduino ESP32 cores based on ESP-IDF 4, this driver reserves channels 0–3
in Adafruit's reservation table when `MOTOR_SHIELD_TYPE` is defined, matching
`DCCRMT.cpp`. This uses an internal symbol in the pinned library version;
recheck this integration before upgrading Adafruit NeoPixel. Chips with only
four TX channels cannot run this driver alongside that DCC implementation and
are reported offline. ESP32 node builds without DCC do not need the reservation.
Other RMT users must also coordinate channel ownership. IDF 5 uses Adafruit's
Arduino RMT backend; this does not add IDF 5 support to the DCC waveform driver.
The Adafruit `show()` API does not return transmission errors.

Connect GPIO to the string's data input, provide the correct LED supply and
connect the ESP32 and LED supply grounds. Follow your strip's requirements for
logic level conversion and power distribution. Hardware validation should
include LED updates while DCC is running and checking command responsiveness
at the intended string length.
