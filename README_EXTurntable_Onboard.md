# Onboard EX-Turntable

The `EXTurntable_OB` device drives a step/dir stepper driver directly from the
CommandStation. Existing EXRAIL `EXTURNTABLE` and `MOVETT()` definitions remain
unchanged.

Pin defaults follow the EX-Turntable board definitions:
ESP32 uses step/dir/enable pins 17/16/18, home sensor 35, limit sensor 34, relay 27, LED 32 and accessory 33

ESP32-S3 uses step/dir/enable pins A0/A1/A2, home sensor D5, limit sensor D8, relay D4, LED D6, and accessory D7.


RT_EX-Turntable board uses:

Arduino Nano ESP32-S3 step/dir/enable pins A0/A1/A2, home sensor D5, limit sensor D8, relay D4, LED D6 and accessory D7.
Additional inputs/outputs D9, D10, D11 and D12.


# myPlatformio.ini file

Add an environment to your myPlatformio.ini file.  I using other that arduino nano esp32 then modify 
sections as required.

```
[env:EX_Turntable_Onboard]
platform = espressif32
board = arduino_nano_esp32
framework = arduino	

; Board-specific settings (ESP32-S3 with 16MB Flash, 8MB PSRAM)
board_build.mcu = esp32s3
board_build.f_cpu = 240000000L
board_build.f_flash = 80000000L
board_build.flash_mode = dio
board_build.partitions = default_16MB.csv

lib_deps = 
	waspinator/AccelStepper @ ^1.64.0


; Build flags for PSRAM and partition support
build_flags =
    -std=c++17 
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DBOARD_HAS_PSRAM
	-DDCCEX_NODE
    -DDCCEX_NODE_NANO_EXTURNTABLE
    -DEXTURNTABLE_ONBOARD
    -DNO_DISPLAY
	-DI2C_SDA=11
	-DI2C_SCL=10
    -mfix-esp32-psram-cache-issue

; Specify upload and monitor speed
upload_speed = 921600
upload_protocol = esptool
monitor_speed = 115200
monitor_echo = yes
monitor_dtr = 0
monitor_rts = 0
```



The onboard driver uses a TMC2209 stepper motor driver.

 `waspinator/AccelStepper` is included as a PlatformIO dependency.


Other boards use the non-ESP32 assignments from EX-Turntable.  Although this is not really applicable for a DCC-EX node.


Pins can be overridden in
`config.h` with
`EX_TURNTABLE_STEP_PIN`
`EX_TURNTABLE_DIR_PIN`
`EX_TURNTABLE_ENABLE_PIN`
`EX_TURNTABLE_HOME_SENSOR_PIN`
`EX_TURNTABLE_LIMIT_SENSOR_PIN`
`EX_TURNTABLE_RELAY_PIN`
`EX_TURNTABLE_LED_PIN`
`EX_TURNTABLE_ACCESSORY_PIN`

If `EX_TURNTABLE_FULL_STEP_COUNT` (or the existing `FULL_STEP_COUNT`) is not
defined, the controller homes and calibrates at startup.
The measured full-turn step count is saved in NVS when it fits the NVS signed 16-bit value range.

Define `EX_TURNTABLE_MODE_TRAVERSER` for traverser operation/
Define `EX_TURNTABLE_MANUAL_PHASE_SWITCHING` to disable automatic phase switching.


## myAutomation file

Add

`HAL(EXTurntable_OB, 600, 1)`


Other commands are as in the EX-Rail command reference.


# First start up

The driver will attempt to move the turntable to home and then calibrate the full step count.

If the home sensor can't be found then the calibration will not occur.






## NVS configuration

The onboard controller reads its configuration from the DCC-EX node NVS table
when its IO device starts. NVS(0) contains the EX-Turntable's virtual pin
address. When this value is nonzero, the HAL device is created at that vpin;
otherwise, the vpin provided in the `HAL(EXTurntable, ...)` declaration is
used. Configuration values use the fixed NVS IDs shown below; NVS(0) is not a
base address for those settings.

Use `<C NVS id value>` to set a value. An unset or zero value uses the firmware
or `config.h` default. Pin, speed, timing, and count settings require a positive
value. The NVS table stores signed 16-bit values, so a full step count saved
after calibration must be 32767 or less; larger calibrations remain available
until restart and should instead be supplied through `FULL_STEP_COUNT` or
`EX_TURNTABLE_FULL_STEP_COUNT`.

| NVS ID | Setting | Value |
| --- | --- | --- |
| 0 | EX-Turntable address | Virtual pin (vpin) where the device is located |
| 1 | Step pin | Step/ pulse output pin |
| 2 | Direction pin | Direction output pin |
| 3 | Enable pin | Stepper driver enable pin |
| 4 | Home sensor pin | Home sensor input pin |
| 5 | Limit sensor pin | Traverser limit input pin |
| 6 | Relay pin | Track phase relay output pin |
| 7 | LED pin | Status LED output pin |
| 8 | Accessory pin | Accessory output pin |
| 9-12 | Extra output pins 1-4 | Outputs used by activities 10-17 |
| 13 | Maximum speed | AccelStepper maximum speed |
| 14 | Acceleration | AccelStepper acceleration |
| 15 | Full-turn step count | Calibrated or configured full-turn steps |
| 16 | Sanity steps | Maximum travel limit used for homing/calibration |
| 17 | Home sensitivity | Minimum calibration travel before accepting home |
| 18 | Phase-switch angle | Automatic phase switch angle in degrees (1-179; zero uses the default) |
| 19 | Gearing factor | Multiplier for commanded positions (1-10) |
| 20 | Home sensor active state | 1 = LOW, 2 = HIGH |
| 21 | Limit sensor active state | 1 = LOW, 2 = HIGH |
| 22 | Relay active state | 1 = LOW, 2 = HIGH |
| 23 | Sensor debounce | Debounce time in milliseconds |
| 24 | Slow LED interval | LED blink interval in milliseconds |
| 25 | Fast LED interval | LED blink interval in milliseconds |
| 26 | Operating mode | 1 = turntable, 2 = traverser |
| 27 | Rotation mode | 1 = shortest path, 2 = forward only, 3 = reverse only |
| 28 | Phase switching mode | 1 = automatic, 2 = manual |
| 29 | Invert direction | 1 = enabled, 2 = disabled |
| 30 | Invert step | 1 = enabled, 2 = disabled |
| 31 | Invert enable | 1 = enabled, 2 = disabled |
| 32 | Keep stepper outputs enabled at idle | 1 = enabled, 2 = disabled |
| 33 | Extra outputs enabled | 1 = enabled, 2 = disabled |

Boolean settings use 1/2 rather than 1/0 because the NVS table does not retain zero-valued entries.

After calibration, the controller stores the measured full step count in setting 15 when it fits the NVS maximum 32767

