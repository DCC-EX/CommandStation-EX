# Handling signals in a node

This example shows how an existing signal definition  can be moved to a node processor. Note that this includes th new signal description feature.

## Existing EXRAIL for signal connected to Command Station

```cpp
SIGNALH(22,23,24,"Mountain pass")
```

This is an awkward situation because the SIGNAL and SIGNALH macros assume that the signal id is the same as the red vpin. When we move the signal to the Node, we want to keep the signal id (to share between nodes) but use different vpins according to the HAL on the node.
To make this easier and new LED_SIGNAL mcro has been added that separates the signal id from the vpins:

```cpp
LED_SIGNAL(123,22,23,24,"Mountain pass")
```

This gives a signal with ID 123..

- full EXRAIL support for RED/AMBER/GREEN, IFRED etc, ONRED etc and WAIT_ON_RED.
- New  throttle support for signals. (refer ```<JS>```)

## Signal Moved to remote node

This involves moving the hardware-aspect of the signal definition to the myAutomation.h in the node..

```cpp
// Signal definition in node copy of myAutomation.h
LED_SIGNAL(123,200,201,202,"Mountain pass")
```

Notice that the signal ID does not change but the vpins may well change to suit whatever HAL device is driving the signal.

The Command Station will automatically know that the signal is defined elsewhere so does not need the signal definition.

This is particularly powerful if you are building a modular layout. If for eaxample a bunch of signals are on a particular module handled by a node, then the throttles will automatically know if the module is present or absent and will not show signals for the module unless it is powered up.

Notice that this is all you need to do... Any EXRAIL in the Command station that needs to handle the signal can remain unchanged.

You can have any number of signals in a node or split them over several nodes, and still manage some from the CS if needed..

In effect, signal numbers are unique across all nodes so only one node should have the hardware-aware signal definition.

## DCC Signals

FOR THE TIME BEING a DCC signal can't be moved to a node because its "wired" to the CS/Track.

That doesn't stop a node throwing or closing it as the id is shared automatically between all nodes.

## Local node processing

Because the node has almost the full (except train driving.. maybe later) EXRAIL capability, its possible to use EXRAIL in the node to monitor ONRED etc to perform local signals, switch lights, make sounds and so on.
