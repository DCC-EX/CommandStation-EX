# Use of Non Volatile Storage

Non Volatile STorage (NVS) exists to enable a command staton or node to be configured easily without involving a recompile or flash.

This is particularly useful for:

- Wifi Settings 
- OLED type settings
- Configuration of factory created nodes which provide standardised turnouts and signals but require individual confuguration for things like a base address, descriptions, servo positions.
- Configuration of user or system provided EXRAIL scripts for things like power-on-at-boot etc.

## NVS Storage

The system provides for 256 NVS values. Each of which defaults to 0 but may be configured as a nummber (-32768..+32767) or a string value.

These values are maintained over power down.

## Access to values (Command line)

Values may be set by the ```<C NVS``` command as follows:

```cpp
<C NVS 29 333>

```
Sets NVS(29) to the value 333

```cpp
<C NVS 77 "My Big Red SIgnal">
```

Sets NVS(77) to a string value.

NVS values can be displayed by the command ```<D NVS>``` which will list all the non-xero NVS values as ```<C NVS``` commands.

## Access to values (in EXRAIL)

### Numeric values

EXRAIL scripts may access NVS numeric values in a variety of ways. Some of these methods will only take the NVS value at startup time, other methods will self modify at run time if the NVS value is changed without restarting.

Numeric NVS values may be accessed using the NVS(id) macro. This is best seen by some typical examples.

```cpp
DELAY(NVS(66))
SETLOCO(NVS(17))
SPEED(NVS(18)) 
```

In commands that are "executed" in sequences, like the above, NVS values will be extracted at execution time, so a change to the NVS(66) value will take effect next time that DELAY is executed.

However EXRAIL macros that define things at statup, will only examine the NVS value at startup... For example:

```cpp
SERVO_TURNOUT(1,100,NVS(21),NVS(22), Slow,"My turnout")
```

will evaluate the values in NVS(21) and NVS(22) once only at startup.

[Its accepted that this is not great for servo settings, it may be possible to modify servo handling to reevaluate these at run time but wait and see]
 
This can be particularly tricky when using a turnout or signal id because the turnout definition, or ONTHROW/ONCLOSE  will be evaluated once at startup but a THROW will be re-evaluated each time it is executed.

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

## Creating browser dialogs to edit NVS values

The serial log browser inteface, which allows a user to monitor the serial log and issue commands from a web browser pointed at the hostname or ip-address of the CS or Node, contains an integrated configuration editor feature which allows a user (or factory) dialog to be attached.

Dialogs can be exgtreemly simple or staggeringly complicated, such is the world of HRTML and JavaScript. However, simple dialogs can be created with a minimum of red-tape as is the philosophy behind EXRAIL. 

TO BE CONTINUED
