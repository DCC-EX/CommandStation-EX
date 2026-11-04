# Sensors and VPINs on nodes

COMPLETE REWRITE - FOR DISCUSSION AS SENSOR SHARING EVOLVES - NOT YET IMPLEMENTED  
BEWARE... this is changing and may be different to what you had, or what you get!

Sharing sensors is an obvious use for nodes but is complicated by the use of vpin numbers which might overlap between nodes.

Remember, you dont have to do this for Turnouts or Signals (and hopefully turntables when implemented) because they already have their own id ranges and can be synchronized automatically without the need for remapping.

## What is a "sensor"

In this case we are referring to a software process that polls a particular vpin, does debouncing etc and informs the system when it changes. We do not care what is on the far end of a vpin, it may be software or hardware. The polling process is the same for all kinds of uses, but there are differences in expectation when changes are detected.  
  
- Something (perhaps JMRI or similar PC program or a throttle app like Engine Driver or EXToolbox) may be expecting a '<Q/q>' type indication over the throttle interface (USB serial, wifi sockets, UDB broadcast etc).
- EXRAIL may be using an ONBUTTON, ONBUTTONUP or ONSENSOR handler.
- Other nodes may need to be told

There are various sensor definition macros in EXRAIL for this.

NEW: there are origin parameters 

`JMRI_SENSOR(vpin[,count[,newOrigin]])`


```cpp
JMRI_SENSOR(100) // = JMRI_SENSOR(100,1,100)
JMRI_SENSOR(100,8) // = JMRI_SENSOR(100,8,100)
JMRI_SENSOR(100,8,2000) 
or 
JMRI_SENSOR_NOPULLUP(....)  // as above rarely used, frequently misunderstood
```

This sensor monitors vpins  and if they change, a notification ( <Q/q> ) is sent to the command station USB serial and other dcc-ex protocol throttles but with the vpin numbers from the newOrigin. As per all previous versions. Sensors created at run time with the `<S` commands will create the same type of sensor.
e.g.

```cpp
JMRI_SENSOR(400,9,9000)
```

will detect a change on vpin 403 and send `<Q/q 9003>`.


NEW: If JMRI_SENSOR is used in a node, the node will inform the command station and the CS will pass the change on to all `<>` protocol jmri/throttles (using the well loved `<Q/q vpin>`) as if it had been detected on the CS. The CS, and indeed other nodes, will also see this as a shared sensor so any node using REMOTE_SENSOR can pick this up and react.

## NEW: Sensors on nodes

In order for a Node or CS to monitor vpins which are to be listened for on other nodes or the CS (but not necessariliy passed on to throttles.) We must tell the node to broadcast a sensor change to other nodes.
[NOTE: this is the same as JMRI_SENSOR but the recipient does not automatically broadcast it to all throttles]

It is generally better to use the newOrigin parameter to avoid trying to share vpins that are common to multiple hardware setups. 

```cpp
SHARED_SENSOR(164,8,2000)
SHARED_SENSOR_NOPULLUP( ... ) is also available with the same parameters  
```

This sensor monitors vpins 164 to 171 and if they change, a notification is sent to ALL nodes (including the command station) but using the vpins 2000..2007.  Each node may be listening or not and it's important that nodes that are not listening for the change do not get confused by it.

For a node or the CS to listen to NODE_SENSOR triggers it must create its own listening sensors:

```cpp
REMOTE_SENSOR(9000)  // = REMOTE_SENSOR(9000,1)
REMOTE_SENSOR(9000,8)
```

This construct creates a virtual vpin (Bitmap probably) that can be listened to and tested by EXRAIL using all the usual constructs.

```cpp
 ONBUTTON(9000)  do something
 ONBUTTONUP(9000) do something
 ONSENSOR(9003)  do something
 IF(9003)
 AT(9003) 
 and so on
 ```


## Worked example:

   A mimic panel node wishes to listen for block occupancy sensors on another node so that it can manipulate some lights.

   The block-occupancy node has something like this:

   ```cpp
   HAL(MCP23017,500,16) // actual sensor hardware
   SHARED_SENSOR(500,16,8000) // share these pins as 8000..8015 with all nodes.
   ```

The mimic panel node does this:

   ```cpp
   REMOTE_SENSOR(8000,16)  // listen for other nodes telling us about these

   ONBUTTON(8003)  // block 3 occupied
     NEOPIXEL(703,255,0,0,4)  // light up the 4 pixel path in red
     DONE
   
   ONBUTTONUP(8003)  // block 3 unoccupied
     NEOPIXEL(703,60,60,60,4)  // light up the 4 pixel path in grey
     DONE
   ```

## Factory built TOTI example

Its generally better to use unique vpins where you can but for example a TOTI node shipped ready built will probably have fixed GPIO vpins because of the hardware design but will need to have an editable NVS value to set the new origin of the sensor transmission, or even whether to use SHARED_SESNOR or JMRI_SENSOR...

Something like :

```cpp
JMRI_SENSOR(  10, NVS(1)?8:0, NVS(2))
SHARED_SENSOR(10, NVS(1)?0:8, NVS(2))
```

where NVS(1) is a tick box for JMRI or just shared, and NVS(2) is the base id sent in the Q msgs. (a zero count means no sensors created)

 ## Sharing vpin writes with other nodes (STILL THINKING ABOUT THIS!)  

Shared writing of vpins is a powerful technique because it allows features like PLAY_TRACK, or NEOPIXEL  to be invoked on one node but executed on another.

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
