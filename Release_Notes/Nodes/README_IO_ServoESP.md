# IO_ServoESP

`IO_ServoESP` drives hobby servos directly from ESP32 GPIO pins using the
ESP32 LEDC PWM peripheral; it does not require a separate PWM controller.
The driver is compiled for ESP32 builds with `DCCEX_NODE` defined, provided
`MOTOR_SHIELD_TYPE` is not defined. It is included automatically through
`IODeviceList.h` in those builds.

## Define the device

Add a `HAL` declaration in `myAutomation.h`. The first number is the first
virtual pin (VPIN), the second is the number of servos, and the final
initializer list gives the ESP32 GPIO used for each VPIN, in order:

```cpp
HAL(ServoESP, 100, 2, {13, 23})
```

This assigns VPIN 100 to GPIO 13 and VPIN 101 to GPIO 23. The number of GPIOs
in the list must match the number of VPINs, and a GPIO cannot be repeated.
The GPIOs may be non-consecutive. To use consecutive GPIO numbers, the
shorter form is also supported:

```cpp
HAL(ServoESP, 100, 2, 13)  // VPIN 100 -> GPIO 13; VPIN 101 -> GPIO 14
```

The driver supports up to four servos by default (`MAXSERVOS`); builds may
override that limit. Each servo uses one VPIN and one distinct GPIO. The
default LEDC channel range starts at channel 8. An optional fourth argument
to the initializer-list form can set the first LEDC channel; leave it at its
default unless you have checked that it does not conflict with other LEDC
users.

## Available output GPIOs

The driver validates GPIOs against this exact list:

**GPIO 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33**

Only these pins can be selected, even if another pin on a particular ESP32
board is capable of digital output. Check your board's pinout and any board
peripherals before wiring a servo; some listed GPIOs may be used by other
hardware on a specific board.

For example gpio21 and 22 are the usual i2c pins for a ESP32-Wroom.

NOTE:   The GPIO supplies the PWM signal only; power the servo from a suitable
external supply, and connect the supply ground to ESP32 ground. Do not power
the servo from an ESP32 GPIO.


## Use ServoESP from EXRAIL

Define the `ServoESP` HAL in `myAutomation.h`, then use its VPINs with the
EXRAIL servo or signal commands. Each VPIN must be within the HAL's VPIN range:

```cpp
HAL(ServoESP, 100, 4, {13, 23, 25, 26})
```

### Servo turnout

Define a turnout using the VPIN connected to its servo:

```cpp
SERVO_TURNOUT(1, 100, 600, 300, Medium, "Yard turnout 1")
SERVO_TURNOUT(2, 101, 600, 300, Medium, "Yard turnout 2")
```

This creates turnout ID 1 on VPIN 100 (GPIO 13) and ID 2 on VPIN 101
(GPIO 23). The third and fourth arguments are the thrown/active and
closed/inactive PWM positions, respectively. They are hardware-dependent
values in the range 0–4095, not angles in degrees. Calibrate them for the
turnout mechanism. The profile controls movement time and may be `Instant`,
`Fast`, `Medium`, or `Slow`.

The turnout starts in its closed/inactive position when initialized. After
initialization, use the turnout ID in EXRAIL commands such as `THROW(1)` and
`CLOSE(1)`, or operate it through a throttle. Use a different turnout ID for
each turnout, and define each turnout against the VPIN connected to its servo.

### Servo signal

`SERVO_SIGNAL` assigns one ServoESP VPIN to a signal. Its first argument is
both the signal ID and servo VPIN; the next three arguments are the PWM
positions for red, amber, and green aspects, followed optionally by a quoted
description:

```cpp
SERVO_SIGNAL(102, 600, 1800, 3000, "Yard signal")
```

The example uses VPIN 102 (GPIO 25). Use `RED(102)`, `AMBER(102)`, and
`GREEN(102)` in EXRAIL to select the corresponding positions. These commands
write the positions immediately (no movement profile); calibrate each value
for the signal mechanism. Positions are clamped to 0–4095.

### Direct servo commands

Use `SERVO` or `SERVO2` inside an EXRAIL route to write a position directly
to a VPIN:

```cpp
AUTOSTART
  SERVO(103, 600, Fast)
```

`SERVO(vpin, position, profile)` moves the servo on that VPIN to the position
using the named profile. The position is a PWM value from 0 to 4095, not an
angle. Supported profiles for ServoESP are `Instant`, `Fast`, `Medium`, and `Slow`.
`Instant` moves immediately; the other profiles take approximately 500 ms,
1 second, and 2 seconds, respectively.

`SERVO2` specifies a movement duration instead of a profile:

```cpp
AUTOSTART
  SERVO2(103, 300, 750)
```

This commands VPIN 103 to position 300 over approximately 700 ms. The
duration is converted to units of 100 ms (integer division), so a requested
750 ms is sent as 700 ms. These direct commands do not change a servo's
configured active/inactive positions.


## NOTE
The GPIO supplies the PWM signal only; power the servo from a suitable external supply, 
and connect the supply ground to ESP32 ground. DO NOT power the servo from an ESP32 GPIO.


