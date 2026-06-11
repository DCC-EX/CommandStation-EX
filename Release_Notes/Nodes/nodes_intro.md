# Introduction to using distributed layout nodes

Layout nodes offer a simple way of distributing layout control (usually wirelessly) over a wide area using the same DCC-EX software in each node.

## Basic Principles

A layout may be controlled entirely by a single processor, known as the Command Station, which performs many tasks simultaneously:

- Communicating with throttles
- Controlling and generating the track output
- Running EXRAIL scripts
- Handling accessories
- Displaying on oleds

In some cases, particularly large layouts,  it would be preferable to offload some of these functions to separate processors, mostly for ease of wiring... performance is rarely an issue.

By using the node system, the separate processors will automatically intercommunicate and will use the same DCC-EX code, including EXRAIL and HAL drivers in each node.

For example, a node may be established to handle a group of turnouts on one area of your layout. The physical turnouts would be connected to the node using the usual hardware (MCP32017, PCA9685 etc) and usual EXRAIL definitions but there would not be any need to wire the node to the command station or deal with long i2c connections.

The Command Station would still throw/close the turnout in the same way as normal, but the remote node would perform the physical movement.

## Shared objects

The following object types are shared amongst the nodes but in general a noce/cs that wants to see/alter an object on another node must express an interest in that object.

- Turnouts can be thrown, closed or monitored from any node.
- Signals can be set or monitored from any node 
- EXRAIL RESERVE/FREE tokens can be obtained by any node
- Input (sensor) vpins can be monitored by any node
- Output vpins can be monitored by any node 

## Node processors

Because the nodes do not have to handle the creation of the DCC waveform, or the weird and wonderful assortment of throttle connections, a node can be created using a much cheaper processor than the Command Station and without the motor shield requirement.

However, a CSB1 being used as a booster is effectively a node and can handle other devices as well.

## Node configuration

Every effort has been made to minimise the configuration/scripting changes but this is best seen by viewing the cookbook entries that follow.

In particular, there is no visible concept of a node id or targetting commands to a particular node.

It is assumed that the reader intending to implement nodes is familiar with the Command Station way of defining and using devices with EXRAIL.

## Flashing Nodes

Each node will need its own myAutomation.h but otherwise operates exactly the same EX-CommandStation code. A definition (or possibly runtime switch) prevents the node from advertising itself to throttles or trying to generate a DCC waveform.

(Installer assist in managing this?)