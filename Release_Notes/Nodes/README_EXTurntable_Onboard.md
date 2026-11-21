# EX-Turntable Onboard

The `EXTurntable_OB` device drives a step/dir stepper driver directly from the Command Station.
Existing EXRAIL `EXTURNTABLE` and `MOVETT()` definitions remain unchanged.


Pin defaults follow the EX-Turntable definitions:

ESP32 uses step/dir/enable pins 17/16/18, home sensor 35, limit sensor 34, relay 27, LED 32 and accessory 33

ESP32-S3 uses step/dir/enable pins A0/A1/A2, home sensor D5, limit sensor D8, relay D4, LED D6, and accessory D7.

RT_EX-Turntable board uses:

Arduino Nano ESP32-S3 step/dir/enable pins A0/A1/A2, home sensor D5, limit sensor D8, relay D4, LED D6 and accessory D7.
Additional inputs/outputs D9, D10, D11 and D12.


# myPlatformio.ini file

Add an environment to your myPlatformio.ini file.  If using other that arduino nano esp32 then modify 
sections as required.

```
[env:EX_Turntable_Onboard]
extends = env:ESP32-NODE
platform = espressif32
board = arduino_nano_esp32

build_flags = 
    ${env:ESP32-NODE.build_flags} 
    -std=c++17
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
	-DSD_CHIP_SELECT_MODE=1
    -DBOARD_HAS_PSRAM
    -DEXTURNTABLE_ONBOARD
    -DNO_DISPLAY
    -DINVERT_ENABLE
	-DI2C_SDA=11
	-DI2C_SCL=10
    -mfix-esp32-psram-cache-issue

lib_deps = 
    ${env:ESP32-NODE.lib_deps} 
	waspinator/AccelStepper @ ^1.64.0

; Board-specific settings (ESP32-S3 with 16MB Flash, 8MB PSRAM)
board_build.mcu = esp32s3
board_build.f_cpu = 240000000L
board_build.f_flash = 80000000L
board_build.flash_mode = dio
board_build.partitions = default_16MB.csv

; Specify upload and monitor speed
upload_speed = 921600
upload_protocol = esptool
monitor_speed = 115200
monitor_echo = yes
monitor_dtr = 0
monitor_rts = 0
```



The onboard driver uses a TMC2209 stepper motor driver.  This is the recommended driver module and is
included on an RT_EX-Turntable board.

The TMC2209 stepper motor driver needs enable inverted (active low).  Driver now defaults to true for this.  If you need the enable to be active high comment out `-DINVERT_ENABLE` in the above myPlatformio.ini env.


 `waspinator/AccelStepper` is included as a PlatformIO dependency.


Other boards use the non-ESP32 assignments from EX-Turntable.
Although this is not really applicable for a DCC-EX node.


# Other Defines

The easiest way to overide any of the following it to `#define` them in the `config.h` file


Pins can be overridden with
`EX_TURNTABLE_STEP_PIN`
`EX_TURNTABLE_DIR_PIN`
`EX_TURNTABLE_ENABLE_PIN`
`EX_TURNTABLE_HOME_SENSOR_PIN`
`EX_TURNTABLE_LIMIT_SENSOR_PIN`
`EX_TURNTABLE_RELAY_PIN`
`EX_TURNTABLE_LED_PIN`
`EX_TURNTABLE_ACCESSORY_PIN`


If `EX_TURNTABLE_FULL_STEP_COUNT` (or the existing `FULL_STEP_COUNT`) is not defined, the controller
homes and calibrates at startup.

The measured full-turn step count is saved in NVS when it fits the NVS signed 16-bit value range.


Define `EX_TURNTABLE_MODE_TRAVERSER` for traverser operation.
Define `EX_TURNTABLE_MANUAL_PHASE_SWITCHING` to disable automatic phase switching.


The Extra GPIO pins are dependent on the ESP32 being compiled for.

The pins can be overridden by
`EXTT_DEFAULT_EXTRA_PIN_1`
`EXTT_DEFAULT_EXTRA_PIN_2`
`EXTT_DEFAULT_EXTRA_PIN_3`
`EXTT_DEFAULT_EXTRA_PIN_4`

They need to be ESP32 gpio pins and there is no check to their validity.  The default pins assigned to these are safe to use.


The active state of the home and limit sensors can be set by:

`EX_TURNTABLE_HOME_SENSOR_ACTIVE_STATE`
`EX_TURNTABLE_LIMIT_SENSOR_ACTIVE_STATE`

default is low for both of these


The active state of the relay

`EX_TURNTABLE_RELAY_ACTIVE_STATE`

default is active high


Turntable full step count, this can be set on the brower configuration page.  Not really needed as calibration function does this.

`EX_TURNTABLE_FULL_STEP_COUNT`

default 0




The following can all be set via the browser configuration page.


The maximum speed for the stepper motor

`EX_TURNTABLE_MAX_SPEED`

defalut is 200, for a direct drive stepper motor this is a scale realistic value if you are using a geared turntable you probably want to increase this.


The acceleration rate for the stepper motor

`EX_TURNTABLE_ACCELERATION`

default is 25, this is the rate of change for the stepper motor to increase/decrease speed.


Sanity steps

`EX_TURNTABLE_SANITY_STEPS`

default is 10000, if you are using micro stepping of 1/128 or higher you need to change this.  Suggest 30000 for 1/128


Home sensor sensitivity is the number of steps the motor will move to clear the home sensor.  Important for homing and calibration when the turntable is already at the home sensor.

`EX_TURNTABLE_HOME_SENSITIVITY`

default 300, if you are using 1/128 or higher micro stepping then increse this.


Phase switching angle for auto phase switching.  Not normally needed to be changed.

`EX_TURNTABLE_PHASE_SWITCH_ANGLE`

default 45 


For geared turntables, this is included as it is part of the original EX-Turntable.  It's probably nolonger required as the calibration should calculate the correct number of steps.

`EX_TURNTABLE_GEARING_FACTOR`

default 1


Led slow flash rate in milliseconds

`EX_TURNTABLE_LED_SLOW_MS`

default 500mS


Led fast flash rate in milliseconds

`EX_TURNTABLE_LED_FAST_MS`

default 100mS


Home/Limit sensor debounce time in milliseconds

`EX_TURNTABLE_DEBOUNCE_MS`

default 0  for turntable mode
default 10  for traverser mode


Traverser mode

`EX_TURNTABLE_MODE_TRAVERSER`

default not defined


Phase switching, if you want to manually switch the turntable phase so that EX-Rail, etc controls the switch.

`EX_TURNTABLE_MANUAL_PHASE_SWITCHING`

default not defined


Turntable move forward (clockwise) only

`EX_TURNTABLE_ROTATE_FORWARD_ONLY`

default not defined


Turntable move reverse (anti-clockwise) only

`EX_TURNTABLE_ROTATE_REVERSE_ONLY`

default not defined




## myAutomation file

Add

`#include "IO_EXTurntable_Onboard.h"`
`HAL(EXTurntable_OB, 600, 1)`

Other commands are as in the EX-Rail command reference.

Additional activity commands are

10  -   Extra 1 on
11  -   Extra 1 off
12  -   Extra 2 on
13  -   Extra 2 off
14  -   Extra 3 on
15  -   Extra 3 off
16  -   Extra 4 on
17  -   Extra 4 off



# First start up

The driver will attempt to move the turntable to home and then calibrate the full step count.

If the home sensor can't be found then the calibration will not occur.

After successful calibration, the controller stores the measured full-turn step count in NVS(15)
when it fits the NVS maximum of 32767.

It then generates default positions for 48 tracks in NVS(40)-NVS(87): each entry is the previous
position plus `NVS(15) / 48`, so NVS(40) is one interval and NVS(87) is 48 intervals.
These generated values replace any existing values in those slots.

This is equivalent to a track angle of 7.5 degrees.

This will also occur any time a calibration is redone.

**NOTE:**  Once calibration is completed the node will restart after 20 seconds to reload all the relevant NVS values and rehome the turntable.



# Subsequent start up

The controller will move the turntable/traverser to the home position.


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
| 0 | EX-Turntable address | id for the turntable in EX-Rail |
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
| 34 | Turntable description | Description used in EX-Rail |

Boolean settings use 1/2 rather than 1/0 because the NVS table does not retain zero-valued entries.

| NVS ID | Setting | Value |
| --- | --- | --- |
| 40-87 | Track positions | Stepper positions for tracks 1-48, generated after calibration |
| 90-137 | Track descriptions | Description text for tracks 1-48 |


# EXRail and other control.

See here as well

https://dcc-ex.com/ex-turntable/test-and-tune.html#ex-turntable-activity-reference

For commands other than activity 0 or 1.  The steps must be 0

eg.

To turn the led on fast flash at the serial console using the turntable diagnostic command and id 600

```<D TT 600 0 6>```

In EXRail this would be 

```ROTATE(600, 0, 6)```

