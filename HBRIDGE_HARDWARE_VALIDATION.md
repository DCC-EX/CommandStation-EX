# Kato/H-bridge turnout hardware validation

Verify that the H-bridge provides current limiting or equivalent coil protection. Never connect a Kato turnout directly to a microcontroller pin.

Record the CommandStation board and firmware commit, H-bridge part, supply voltage, Kato turnout model, distinct throw/close VPINs, configured pulse, and measured pulse width.

Acceptance criteria:

1. `<T id HBRIDGE throwVpin closeVpin pulseMillis>` creates and reports the turnout.
2. Throw raises only the throw output; close raises only the close output.
3. Both outputs return LOW after the pulse, with no sustained HIGH output.
4. A requested pulse above 500 ms is limited to 500 ms or less.
5. Repeated throw/close commands never leave both outputs HIGH.
6. Sensor reports, programming-track operations, and throttle commands continue responding during actuation.
7. Standard DCC, VPIN, and servo turnouts still operate normally in the same build.

Measure the H-bridge input with an oscilloscope or logic analyser. Start at the shortest pulse that reliably changes the turnout, and inspect the driver and turnout for heating during at least 20 repeated throw/close cycles.
