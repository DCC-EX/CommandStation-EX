# ESP32 motor-current ADC attenuation

Issue [#510](https://github.com/DCC-EX/CommandStation-EX/issues/510) reports that ESP32 motor-driver ADC inputs are configured for 11 dB attenuation. The code uses the raw ADC code returned after that configuration and applies the motor-shield-specific `sense_factor` in `MotorDriver::raw2mA()` and `MotorDriver::mA2raw()`.

Attenuation must not be multiplied into the current conversion a second time. Existing `tripMilliamps` values must not be changed from attenuation alone: required electrical constants depend on the shield current-sense circuit, ADC pin protection, supply/reference behavior, and ESP32 ADC calibration.

## Required hardware evidence before changing a threshold

For each ESP32 board and motor-shield definition:

1. Record the exact board, Arduino-ESP32 2.x/IDF 4 version, shield revision, current-sense details, and ADC GPIO.
2. With track power off, record the ADC raw code at zero current after cold start and warm-up.
3. Apply at least three measured loads across the intended range. Record calibrated current, ADC raw code, and attenuation at every point.
4. Confirm the highest intended sense voltage is below the verified 11 dB input range and the raw code does not clip before the documented trip current.
5. Demonstrate the measured trip point is within the maintainer-approved tolerance, then verify an excessive load enters `POWERMODE::OVERLOAD`.

Attach raw measurements and instrument accuracy to any proposed change to `sense_factor`, `tripMilliamps`, `ADCmax()`, or overload thresholds.
