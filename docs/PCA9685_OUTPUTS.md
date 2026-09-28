# PCA9685 output contract

`IODevice::write(vpin, HIGH)` sets a PCA9685 channel to 100% duty (`4095`) and
`LOW` sets it to 0% duty. A digital write is a level operation: HIGH remains
active until LOW is sent. It does not create a pulse or infer coil polarity.

Use an external transistor/MOSFET or suitable solenoid driver; never connect a
coil directly to a PCA9685 output. A solenoid user must arrange a bounded
activation interval and send LOW on every completion and error path. Servo
profiles are motion/power-management behavior, not a coil current limiter.

After initialization, CommandStation-EX explicitly writes 0% duty to every
configured channel. If I2C probing fails, the device enters `DEVSTATE_FAILED`
and writes are ignored. The external driver must still de-energize the load
when the command-station signal disappears.

## Hardware bench acceptance

1. Power-cycle and independently reset the PCA9685; every channel stays off
   until an intentional HIGH.
2. Confirm HIGH is held and LOW removes drive; measure coil current and driver
   temperature, not only I2C registers.
3. Disconnect SDA/SCL, reset the command station, and remove its power. The
   external driver must turn the coil off without firmware help.
4. Exercise the activation interval repeatedly and record maximum on-time,
   release time, worst-case current, and driver temperature against ratings.
5. Test I2C failure/recovery and ensure recovery produces no unintended pulse.

These are electrical acceptance criteria; software timing alone is not a claim
of electrical safety.
