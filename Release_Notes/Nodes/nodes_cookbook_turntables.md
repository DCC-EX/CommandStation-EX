# Handling turntables in a node
????????????????????? INITIAL DRAFT ?????????????
This example shows how an existing turntable definition  can be moved to a node processor. This only applies to the second version of the turntable commands, where turntables have an id. 

## Existing EXRAIL for turntable connected to Command Station

```cpp
HAL(turntable...???? ,"Roundhouse")
```

This provides :

- full EXRAIL support for ?????  commands and turntable throttle controls.

## Turntable Moved to remote node

This involves moving the hardware-aspect of the turntable definition to the myAutomation.h in the node. Unlike on the CS, the turntable driver can exist within the node so no extra processor is required. 

```cpp
// turntable definition in node copy of myAutomation.h
HAL(EXTurntable_Onboard, ?????????)
```

Notice that the turntable ID does not change but the vpin ???? may well change to suit whatever HAL device is driving the turntable.

The Command Station will automatically know that the turntable is defined elsewhere so does not need the turntable definition.

This is particularly powerful if you are building a modular layout. If for eaxample a turntable is on a particular module handled by a node, then the throttles will automatically know if the module is present or absent and will not show turntables for the module unless it is powered up.

Notice that this is all you need to do... Any EXRAIL in the Command station that needs to handle the turntable can remain unchanged.

You can have any number of turntables in a node or split them over several nodes (depending on hardware pin availability), and still manage some from the CS if needed..

In effect, turntable numbers are unique across all nodes so only one node should have the hardware-aware turntable definition.


## Local node processing

Because the node has almost the full EXRAIL capability, its possible to use EXRAIL in the node to monitor ON??? events to perform local signals, switch lights, make sounds and so on.

