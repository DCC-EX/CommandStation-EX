# Use of Non Volatile Storage

Non Volatile Storage (NVS) exists to enable a command staton or node to be configured easily without involving a recompile or flash.

This is particularly useful for:

- Wifi Settings
- OLED type settings
- Configuration of factory created nodes which provide standardised turnouts and signals but require individual confuguration for things like a base address, descriptions, servo positions.
- Configuration of user or system provided EXRAIL scripts for things like power-on-at-boot etc.

## NVS Storage

The system provides for NVS values numbered 0..255. Each of which defaults to 0 but may be configured as a nummber (-32760..+32760) or a string value. (the 0 is deliverate, some values are reserved for internal flagging purposes.)

These values are maintained over power down.

## Access to values (Command line)

Values may be set by the ```<C NVS``` command as follows:

```cpp
<C NVS 29 333>

```
Sets NVS(29) to the value 333

```cpp
<C NVS 77 "My Big Red Signal">
```

Sets NVS(77) to a string value.

NVS values can be displayed by the command ```<D NVS>``` which will list all the non-zero NVS values as ```<C NVS``` commands.

## Access to values (in EXRAIL)

### Numeric values

EXRAIL scripts may access NVS numeric values in a variety of ways. Some of these methods will only take the NVS value at startup time, other methods will self modify at run time if the NVS value is changed without restarting.

Numeric NVS values may be accessed using the NVS(id) macro. This is best seen by some typical examples.

```cpp
DELAY(NVS(66))
SETLOCO(NVS(17))
SPEED(NVS(18))
PLAY_TRACK(7000,NVS(88)) 
```

In commands that are "executed" in sequences, like the above, NVS values will be extracted at execution time, so a change to the NVS(66) value will take effect next time that DELAY is executed.

However EXRAIL macros that define things at statup, will only examine the NVS value at startup... For example:

```cpp
SERVO_TURNOUT(1,100,NVS(21),NVS(22), Slow,"My turnout")
```

will evaluate the values in NVS(21) and NVS(22) once only at startup.

[Its accepted that this is not great for servo settings, it may be possible to modify servo handling as a special case to reevaluate these at run time but wait and see]
 
This can be particularly tricky when using a turnout or signal id because the turnout definition, or ONTHROW/ONCLOSE  will be evaluated once at startup but a THROW, RED etc  will be re-evaluated each time it is executed.

Caution must be observed in IF() type macros because

```cpp
IF(NVS(13)) 
```

Does not test the value of NVS(13), it tests the vpin referenced by the value in NVS(13)

An additional function IFNVS may be used to test a non-zero NVS setting

```cpp
IFNVS(123) 
  PRINT("NVS 123 has a value ")
  SETLOCO(NVS(123))
ELSE
  PRINT("NVS 123 has no value or is zero")
  DONE
ENDIF
```

### Text values

Accesing NVS text values uses the NVST macro which will generate a runtime tag inside a string, such that the string, when displayed will have the NVS value embedded. This means that NVST values will generally work immediately without the need to restart.

NVST may be used stand alone or as part of another string, the following will be valid:

```cpp
SCREEN(0,1,NVST(1))
SCREEN(0,2,"Your name is " NVST(1))
SCREEN(0,3, NVST(1) " is your name")
SCREEN(0,4, "Hi " NVST(1) " have fun!")
```

## Worked example of using NVS values

Suppose we have a simple AUTOMATION sequence like this to run around in circles and stop each time at sensor 101.

```cpp
AUTOMATION(1,"Run around")
  FWD(50)
  AT(101)
  STOP
  SCREEN(0,6,"Stopped at station")
  DELAY(10000)
  FWD(20)
  DELAY(2000) // allow time to clear sensor 
  FOLLOW(1)
```

But we want to make the speed, delays and message configurable.
We can change the Automation like this to use a set of NVS values:

```cpp
AUTOMATION(1,"Run around")
  FWD(NVS(21))
  AT(101)
  STOP
  SCREEN(0,6,NVST(22))
  DELAY(NVS(23))
  FWD(NVS(24))
  DELAY(NVS(25)) // allow time to clear sensor 
  FOLLOW(1)
```

Once compiled, you can test this immediately by using commands to set the NVS values:

```cpp
<C NVS 21 50>
<C NVS 22 "Waiting for Godot">
<C NVS 23 10000>
<C NVS 24 20>
<C NVS 25 5000>
```

Altering the values with the C command will work even while the automation is running.

## Creating browser dialogs to edit NVS values

The serial log browser inteface, which allows a user to monitor the serial log and issue commands from a web browser pointed at the hostname or ip-address of the CS or Node, contains an integrated configuration editor feature which allows a user (or factory) dialog to be attached.

Dialogs can be extreemly simple or staggeringly complicated, such is the world of HTML/CSS/JavaScript. However, simple dialogs can be created with a minimum of red-tape as is the philosophy behind EXRAIL.

User dialogs can be created in my* files and referenced from EXRAIL so that no base code needs to be modified. Follow this example to create a simple dialog to configure the example sequence shown above.

First we need to create a dialog file fith the name format Like this:

```myRunAround.htnl.h```

The `RunAround` part is your choice but the `my` and `.html.h` must be fixed.

In this file create code like this: Notice correct use of ```RunAround``` in the header.

```html
#include <Arduino.h>
String myRunAround_html=R"???(
Loco starting speed <nvsinput nvs=24 min=2 max=127 />
<br>
Loco driving speed <nvsinput nvs=21 min=2 max=127 />
<br>
Delay at station <nvsinput nvs=23 min=1000 max=32000 /> mS
<br>
Delay to clear station sensor <nvsinput nvs=23 min=1000 max=32000 /> mS
<br>
Message to output when train arrives <nvsinput nvs=22 length=20 /> 
)???";
```

Of course this embedded html can contain all kinds of display and logic but the ```<nvsinput``` tags will automatically fill with the current NVS values and automate the Save button handling.

To make this dialog available to the browser interface, use the following defining command in EXRAIL

```cpp
CONFIGURE_DIALOG("Run Around for fun",myRunAround)
```

the title can be anything but the ```myRunAround``` must match the name chosen for the code.

When the Web Browser interface is opened on the command station, the "Configs" button will pull down a menu of available configuration dialogs and you can select your dialog.
[TODO pic1, pic2]

The dialog will populate with the current values. The Save button will appear if you have changed any of them, or you can close without saving.
