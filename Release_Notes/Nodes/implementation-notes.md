# Nodes implementation notes (Internal, not for user docs)

This file describes the code implementation of the Node system

## Multicast basics

Nodes communicate via multicast to a separate address and port from the throttle multicast. Throttles should NEVER see the nodes traffic and the nodes should not see throttle traffic.

The node multicast IP address is fixed so there is no need to configure this. However, in the case where you need to have more that one CS and layout on the same network, it is necessary to give any additional layout CS and its nodes an alternative last byte of the multicast IP address so that the traffic does not get confused between them.
This is done by a config definition:

```cpp
#define NODE_GROUP 1
```

The default is group 254 and does not need to be altered unless you have more than one layout.

## NodeManager.cpp

This file does a setup of a node traffic listener and provides a generic cast function that can be used with formatting as in StringFormatter::send

Incoming Node traffic is passed to the WifiEESP incoming UDP packet queue so that it does not suffer tasking issues. The queue entries are marked as node traffic so that they can be parsed differently.

## DCCEXParser.cpp

This has a new function to parse incoming node messages using a completely separate command set found in NodeCommands.h.
This means node traffic can be whatever we like without interfering with throttles.

## WifiESP32.cpp

When the incoming UDP queue is processed, node messages are passed back to NodeManager for parsing rather than being put through the usual command parser.

## Answering "Why?"

With the new system, the broadcasts to clients from the CS are unchanged. But the inter-node traffic is not for external consumption.

A turnout change for example, has to be sent out by the CS to:

- USB serial
- Any other command serials
- Each Socket user using the < > protocol
- Each Websocket user
- Each Withrottle user (but in a different format)
- Each UDP client that's not able to listen to the throttle multicast (eg ED)
- The throttle mulitcast

Originally we did think that it would be sufficient for the nodes to just listen to the multicast... but that doesnt give the nodes a way of talking back to the CS or to each other, so we have implemented a separate private comms channel for inter-node communication where we can do stuff like sharing turnouts, signal states etc  without having to interact with throttle traffic.

## Example implementation of a pre-programmed 16 turnout node

HW wise, a 16 channel servo node would have an ESP32 (any old type) and PCA9685 servo controller. Running a copy of EX-CommandStation (without a motor shield definition) and usimg EXRAIL to define the 16 SERVO_TURNOUTs with a range of turnout ids.  So not much different to what you might already have except you dont have to program the PCA9685 driver or the Wifi/UDP etc.....

But here's the variable bit... and these will be no different to whatever you are building now:
  
- The node must be able to identify the Wifi SSID/Password to get on to the same network as the other devices. Since we now have a CSB1 way of setting wifi from a command , the node can be configured over USB (or any other viable input such as a browser, throttle or toolbox connected to the nodes AP) with the `<C WIFI "ssid" "password">`  command and it will stick.
(without the Node code, you would have had to code that manually somewhere.)

- It is very convenient to use `<C WIFI HOSTNAME "nodename">` to set the nodes hostname so that you can easily open a browser to http://nodename/ and monitor the node as for the CS.
