# Assorted new feature documenattion

## Sensors
 
 When sensors are defined its actually creating a list of input pins to be checked, and any changes notified to JMRI(or other throttles), EXRAIL ON-handlers or other Nodes.

 The `<S` commands from DCC++ remain to create sensors that report to JMRI ising `<Q` or `<q`

 the EXRAIL `JMRI_SENSOR(firstpin, count)` creates the same sensors as `<S`

 The EXRAIL `NODE_SENSORS(firstvpin,count)` creates sensors that inform other nodes.

 The EXRAIL ONBUTTON, ONSENSOR and ONBUTTONUP commands automatically create sensors to monitor the pins they handle. 

All of the above can share individual sensor objects internally to avoid repetition. It is not an error to define a sensor more than once or to define it for multiple purposes. 

The new command `<D SENSORS>` can be uysed to diagnose how sensors are built up but the output is a diagnostic and not intended for programs/throttles to interpret.

## New EXRAIL commands

ONBUTTONUP(vpin) Acts like ONBUTTON(vpin) and is more convenient than using ONSENSOR and having to check the state with IF.

DCC_SIGNALL is similar to the existing DCC_SIGNAL but uses a LINEAR accessory address rather than addr,subaddr. 

LED_SIGNAL is similar to SIGNAL but provides a signal id separate to the red vpin. Bear in mind that signal ids are shared between nodes but vpins will be local to the device hosting the leds.

CHANGE_DIRECTION changes loco direction (for example in a shuttle) so can back away from a buffer using SPEED without knowing if the loco was running fwd or reverse.

Exrail serial command `</>` has been updated to include a PAUSED indication.

## Node dialog creation

When editing system defined NVS values in a user (Typically Node factory) dialog, the entry may be referred to my a known system name rather than a fixed NVS number.

## Railcom
 
 Railcom cutout has been enabled for the ESP32.

