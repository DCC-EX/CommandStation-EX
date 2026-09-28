# Automatic track power-off

Automatic power-off is opt-in. Define both `AUTO_POWER_OFF_PIN` and
`AUTO_POWER_OFF_TIMEOUT_MS` in `config.h` to select the relay GPIO and a
non-zero millisecond timeout. The default is disabled because neither macro
is defined, so existing installations retain their current behavior.

The relay GPIO is configured as an output and driven `HIGH` after track setup.
The timer is re-armed by every received command, regardless of command type.
When the timeout expires, the GPIO is driven `LOW` once. This cuts the
station's external supply; track power commands and overload/emergency-stop
handling remain unchanged.

This is an inactivity timeout, not an occupancy, current, or emergency-stop
detector. Keep existing motor-shield overload and emergency-stop protections.

## Validation

Run `python tests/test_auto_poweroff_timing.py` for deterministic deadline,
re-arm, one-shot, disabled-default, and `millis()` wraparound checks. CI must
also compile a normal supported PlatformIO target.

Before enabling it on hardware, verify the relay is HIGH after boot, every
command source refreshes it, it goes LOW by the deadline plus one main-loop
scheduling interval, and it does not chatter or retrigger. Verify the relay's
LOW state actually removes station supply, that startup polarity is safe for
the chosen relay board, and that a disabled/default build leaves the GPIO
untouched during a soak test. Keep an independent emergency-stop and overload
path; this timeout is not an occupancy or current detector.
