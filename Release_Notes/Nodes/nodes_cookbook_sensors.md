# Sensors and VPINs on nodes

Sharing sensors is an obvious use for nodes but is complicated by the use of vpin numbers which might overlap between nodes.

Shared writing of vpins is a powerful technique because it allows features like PLAY_TRACK, or NEOPIXEL  to be invoked on one node but executed on another.

Remember, you dont have to do this for Turnouts or Signals because they already have their own id ranges and can be synchronized automatically without the need for remapping.


BEWARE... this is changing and may be different to what you had.

## What is a "sensor"

In this case we are referring to a software process that polls a particular vpin, does debouncing etc and informs the system when it changes. We do not care what is on the far end of a vpin, it may be software or hardware. The polling process is the same for all kinds of uses, but there are differences in expectation when changes are detected.  
  
- Something (perhaps JMRI or similar PC program) may be expecting an indication over the USB serial or other hardware serial connections. 
- Something (as above plus Webthrottle, websockets etc) running on tcp/ip sockets or udp unicast or udp broadcast.
- EXRAIL may be using an ONBUTTON or ONSENSOR handler.
- Another node may need to know

There are various sensor definition macros in EXRAIL for this.

>> CHRIS NOTE: do we really need to separate broadcast and serial sensors?

```cpp
SERIAL_SENSORS(100,8,2000)
```

This sensor monitors vpins 100 to 107 and if they change, a notification ( <Q/q> ) is sent to the command station USB serial and other serial command outputs but with the vpin numbers 2000..2007.
(for backward compatibility the JMRI_SENSOR(vpin,count) macro performs a SERIAL_SENSORS(vpin,count,vpin))

```cpp
BROADCAST_SENSORS(100,8,2000)
```

This sensor monitors vpins 100 to 107 and if they change, a notification ( <Q/q> ) is sent to all throttles that use the dcc-ex protocol. This includes sockets, websockets, udp etc.

```cpp
SHARED_SENSORS(100,8,2000)
```

This sensor monitors vpins 100 to 107 and if they change, a notification is sent to all nodes (including the command station) but using the vpins 2000..2007

```cpp
 ONBUTTON(101)  do something
 ONSENSOR(101)  do something
 ```

 These commands create internal sensor polls that inform EXRAIL of the VPIN change.

 Its not an error to define overlapping sensor groups. The software will generally optimize the merging of the polling cycle.

 ## Sharing vpin writes with other nodes 

 Whan a vpin is written to by software, it may be echoed to the other nodes but only if it is flagged to do this.

 ```cpp
 SHARE_VPIN_WRITES(vpin,count,base)
 ```

 describes a group of vpins which, when changed, will be notified to all other nodes but with a modified base number.

 ## Accepting vpin changes from other nodes

 Although a node may be announcing its own sensor changes, it is not automatic that any individual node will accept that change 

 Firstly, a node must define the vpin changes it is prepared to accept.

 ```cpp
 ACCEPT_SHARED_WRITES(vpin,count)
 ```

 Means that an incoming sensor change (from a SHARED_SENSORS) or a software write to a SHARE_VPIN_WRITES pin will cause a local write to that particular vpin. 

 Any vpin mentioned in ACCEPT_SHARED_WRITES must actually exist in the HAL... but if it does not, a virtual vpin will be created so that the receiving node can perform IF/WAITFOR/ONSENSOR etc on it.

## Scenarios

###  CS provides sensors to JMRI but some are on nodes

In this case, sensors 100..107 are on the CS but sensors 108..115 are coming from a node that is using its own vpins 100..107 but is transmitting them as 7000..7007

```cpp
HAL(MCP23017,100,16,0x20)
SERIAL_SENSORS(100,8,100)  // normal sensors on my mcp32017

SERIAL_SENSORS(7000,8,108) // sensors on another node
ACCEPT_SHARED_WRITES(7000,8) // Node may send these
```

On the node:

```cpp
HAL(MCP23017,100,16,0x20)
SHARED_SENSORS(100,8,7000)
```

So when pin 101 changes on the node, its transmitted to the CS as 7001 to avoid clash with the CS 101 pin, and then sent on to JMRI as sensor 109.

