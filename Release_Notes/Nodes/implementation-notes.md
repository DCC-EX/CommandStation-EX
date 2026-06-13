# Nodes implementation notes (Internal, not for user docs)

This file describes the code implementation of the Node system. 

## Multicast basics.

Nodes communicate via multicast to a separate address and port from the throttle multicast. Throttles should NEVER see the nodes traffic and the nodes should not see throttle traffic. 

## NodeManager.cpp

This file does a setup of a node traffic listener and provides a generi cast function that can be used with formatting as in StringFormatter::send

Incoming Node traffic is passed to the WifiEESP incoming UDP packet queue so that it does not suffer tasking issues.


## DCCEXParser.cpp

This has a new function to parse incoming mode messages  using a completely separate command set from NodeCommands.h.
This means node traffic can be whatever we like without interfering with throttles.

## WifiESP32.cpp

When the incoming UDP queue is processed, node messages are passed back to NodeManager for parsing rather than being put gthrough the usual command parser.



## Answering "Why?"

With the new system, the broadcasts to clients from the CS are unchanged. But the inter-node traffic is not for external consumption.

A turnout change for example, has to be sent out by the CS to:
 
- USB serial
- Any other command serials
- Each Socket user using the < > protocol
- Each Websocket user 
- Each Withrottle user (but in a different format)
- Each UDP client that's not able to listen to the multicast (eg ED)
- The throttle mulitcast

Originally we did think that it would be sufficient for the nodes to just listen to the multicast... but that doesnt give the nodes a way of talking back to the CS or to each other, so we have implemented a separate private comms channel for inter-node communication where we can do stuff like sharing signal states (trivial)  reserves (complex) without having to interact with throttle traffic.

## Example implementation of a pre-programmed 16 turnout node

HW wise, a 16 channel servo node would have an ESP32 (any old type) and PCA9685 servo controller. Running a copy of EX-CommandStation and usimg EXRAIL to define the 16 SERVO_TURNOUTs turnouts with a range of turnout ids.  So not much different to what you might already have except you dont have to program the PCA9685 driver or the Wifi/UDP etc..... 

But here's the variable bit... and these will be no different to whatever you are building now:
  
- The node must be able to identify the Wifi SSID/Password to get on to the same network as the other devices. Since we now have a CSB1 way of setting wifi from a command , the node can be configured over USB with the <C WIFI "ssid" "password">  command and it will stick.
(without the Node code, you would have had to code that manually somewhere. 

- The node needs to know the turnout ID range that it represents. I dont have a slick solution to this at the moment but I'm sure its going to be easy.