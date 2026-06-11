# Handling turnouts in a node

This example shows how an existing turnout definition  can be moved to a node processor.

## Existing EXRAIL for turnout connected to Command Station

```cpp
SERVO_TURNOUT(22,400,350,120,Slow,"Mountain pass")
```

This provides :

- full EXRAIL support for THROW/CLOSE ONTHROW/ONCLOSE IF etc
- Throttle support for turnout names

## Turnout Moved to remote node

This involves moving the hardware-aspect of the turnout definition to the myAutomation.h in the node..

```cpp
// Turnout definition in node copy of myAutomation.h
SERVO_TURNOUT(22,400,350,120,Slow,"Mountain pass")
```

Notice that the turnout ID does not change but the vpin 400 may well change to suit whatever HAL device is driving the turnout.

AND telling the Command Station that this turnout is somewhere else (with a new command REMOTE_TURNOUT)

```cpp
// Command station version of myAutomation.h
REMOTE_TURNOUT(22,"Mountain pass")
```

The Command Station needs the REMOTE_TURNOUT so that it can still allow EXRAIL and throttles to throw/close, but it doesn't need to know which node has the turnout, nor how it's electronically connected.

Notice that this is all you need to do... Any EXRAIL in the Command station that needs to handle the turnout can remain unchanged.

You can have any number of turnouts in a node or split them over several nodes, and still manage some from the CS if needed..

In effect, turnout numbers are unique across all nodes so only one node should have the hardware-aware turnout definition.

## DCC Turnouts

It must be fairly obvious that a DCC turnout can't be moved to a node because its "wired" to the CS/Track.

That doesn't stop a node declariung a REMOTE_TURNOUT for it and managing that in EXRAIl.

## Local node processing

Because the node has the full (except train driving.. maybe later) EXRAIL capability, its possible to use EXRAIL in the node to monitor ONTHROW/ONCLOSE to perform local signals, switch lights, make sounds and so on.

## Inter-node access

Any node that is prepared to add a REMOTE_TURNOUT to it's myAutomation can THROW, CLOSE, ONTHROW etc without caring which node has the actual turnout. 
