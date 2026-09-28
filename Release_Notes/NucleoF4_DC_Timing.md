# Nucleo-F4 DC-mode timing bench criteria

For two adjacent `DC` districts with one address, verify a locomotive crosses
without a speed step or direction change. Repeat with inverted `DCX` and verify
the expected polarity and no motion while stopped.

Use a two-channel oscilloscope or logic analyzer to confirm equal PWM frequency,
common rising-edge phase after power-on, and no sustained phase drift while
changing speed. Then switch the districts to `MAIN`/DCC and test speed steps,
stop, emergency stop, and power-cycle; DCC timing and rail behavior must remain
unchanged.

Repeat with the actual brake/signal pins on each target variant because timer
mapping differs between Nucleo-F401RE, F411RE, F446RE, and F429ZI. Record board,
STM32 core, shield, pin mapping, PWM frequency, and capture observations.
