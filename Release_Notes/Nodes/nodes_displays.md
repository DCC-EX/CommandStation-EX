# Displays in nodes

Any node may have one or more OLED displays defined in the same way as done now in the command station.

All display traffic (from EXRAIL LCD or SCREEN or `<m screen row "txt">` commands) is relayed to all nodes an so any node that has a HAL definition for a display with the given screen number will automatically update its screen copy.

## Defining an oled

A node may define an OLED screen in exactly the same way as the CS.

More than one node may choose to render the same screen.

it is not necessary to define the oled in the CS if it doesnt have one attached.

## Displaying

The CS or any node may initiate a screen display without having to know where the screen is actually attached.

## JMRI and others

 A screen update by any node will be relayed to the throttle system by the CS so that JMRI or other program can create screen displays.
