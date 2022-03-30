# DCC++EX Track Manager

Chris Harlow 2022/03/23

**If you are only interested in a standard setup using just a DCC track and PROG track, then you DO NOT need to read the rest of this document.**

What follows is for advanced users interested in managing power districts and/or running DC locomotives through DCC++EX.

## What is the Track Manager
Track Manger (TM from now on) is an integral part of DCC++EX software that is responsible for:
- Managing track power state.
- Monitoring track overloads and shorts.
- Routing the DCC main or prog track waveforms to the correct Motor Driver and thus track.
- Managing the JOIN feature.
- Intercepting throttle commands to locos running on DC tracks.
- Handling user or EXRAIL commands to switch track status.

In the default scenario of a single DCC track and a PROG track, the TM behaves as for the previous versions of DCC++EX so if thats what you want, you dont need to mess with it.

The TM is able to handle up to 8 separate track domains. Each domain requires a hardware driver to supply track voltage. A typical motor driver shield supplies two tracks, which is what we have used in the past as main and prog.

Unlike the previous version of DCC++EX, where the shield channel A was always the DCC main and channel B was always the DCC prog track, TM allows :
- None, any or all the tracks can be DCC Main.
- None or ONE track may be DCC prog at any given time.
- Any track may be powered on or off independently of the others.
- Any track may be disconnected from the DCC signal and used as a DC track with a given loco address. (See DC discussion later)

With such flexibility comes responsibility... the potential for making mistakes means taking extra care with your configuration!

**NOTE** TM does NOT use "zero stretching" to control your DC motor. Instead, it uses true Pulse Width Modulation (PWM) to efficiently run your loco using the same method a decoder uses to control a DCC loco's motor. DC locos can even run better on TM than they can on a normal analog throttle, especially at low speed, since it is always applying the full track voltage, albeit in pulses of varying duration. 

## Using the Track Manager (DCC)
TM names the tracks A to H. In a default setup, you will normally have tracks A and B where A will default to be the DCC main signal and B will be the DCC prog.

There is a new user command `<=>` which is used to control the TM but the `<0>` and `<1>` commands operate as before.

- `<=>`  lists the current track settings.
In a default setup this will normally return 
```
        <=A DCC>
        <=B PROG>
```
- `<=t DCC>`  sets track t (A..H) to use the DCC main track.   For example `<=C DCC>` sets track C. All tracks that are set to DCC will receive the same DCC signal waveform.
- `<=t PROG>` Sets track t (A..H) to be the one and only PROG track. Any previous PROG track is turned off.
- `<=t OFF>` turns off the track t. It will not power on with `<1>` because it will not know what signal to send. 

In an all-DCC environment it is unlikely that you will need to do anything other than setting any additional tracks (C...H) as DCC in your `mySetup.h` file.

Bear in mind that a track may actually be only connected to DCC accessories such as signals and turnouts... your layout, your choice. 

Note that when setting a track to PROG or OFF, its power is switched off automatically. (The PROG track manages power on an as-needed basis under normal circumstances.
When setting a track to MAIN (or DC, DCX see later) the power is applied according to the most recent `<1>` or `<0>` command as being the most compatible with previous versions.

## using the Track Manager (DC)

TM allows any or all of your tracks to be individually selected as a DC track which responds to throttle commands on any given loco address. So for example if track A is set to DC address 55, then any throttle commands to loco 55 will be transmitted as DC onto track A and thus a DC loco can be driven along that track. almost exactly as if it was DCC.
Your throttle (JMRI, EX-Webthrottle, Withrottle, Engine Driver etc etc) do not know or care that this is a DC loco so nothing needs to change.

For a simple Command Station setup to run just two DC tracks instead of DCC, you only need to assign DC addresses to tracks A and B. If you want DCC on track A and DC on track B, you just need to set track B to a suitable DC address.

The command to set a track to a DC address is as follows 
- `<=t DC a>` Sets track t (A..H) to use loco address a. e.g. <=A DC 3>

A simple 2 separate loop DC track, wired the traditional way in opposite directions, may be set like this to use loco addresses 1 and 2.  
```
<=A DC 1>
<=A DC 2>
```

### Crossing between DC tracks 

There are some slightly mind-bending issues to be addressed, especially if you want to be able to cross between two separate DC tracks or use your layout in DCC or DC mode. This is because the control of DC loco direction is relative to the TRACK and not the LOCO. (you turn a DC loco round on the track and it continues in the same geographical direction. You turn a DCC loco around and it continues to go forwards or backwards in the opposite geographical direction.)

Generally DC tracks are wired so that two mainline tracks are in opposite direction which makes operation easy BUT crossovers between tracks will cause shorts unless you have very complex switching arrangements.
This is generally incompatible with DCC wiring which expects to be able to cross between tracks with impunity because they are all wired with the same polarity. 

To get over this issue TM allows the polarity of a DC track to be swapped so that tracks wired for DCC may be switched to DC with a polarity chosen at run time according to your operations. So, for example, you may have two loops with a crossing between them. Normally you need them in opposite directions, but when you need to drive over the crossing, you need to switch one or other track so that they are at the same polarity.
(This is a good case for using EXRAIL to help)

The command `<=t DCX a>` will set track t (A..H) to be DC but with reversed polarity compared with a track set to DC.

Its perfectly OK to cross between DC tracks by setting them to the same loco address and making sure you get the polarity right!

## Connecting Hardware
Each track requires hardware to control it
- Power on/off
- Polarity (direction, signal etc)
- Brake (shorts tracks together)
- Current (analog reading)

The standard motor shields provide this for two separate tracks and are predictable and easy to use. However STACKING shields is not a viable way of adding more tracks because it prevents the software from gaining access to the individual track pins. Similarly, wiring all the signal pins together for example, will give you a shared DCC signal but it will eliminate any possibility of switching the track purpose at run time. So, you are going to have to understand enough to wire track drivers to various pins if you wish to extend beyond 2 tracks and take advantage of TM.

You will also need to consider the implications of differing electronic implementations that would cause unexpected issues when a loco moves between tracks. We know this works fine for a typical shield because we use `<1 JOIN>` quite happily but this may be different if you mix hardware types..... (NOT MY PROBLEM !) 

The easiest way to consider the wiring is to treat each track individually (either as a separate driver or as half of a shield).

You will require,for each track, on the Arduino:
- A GPIO pin (or a HAL vpin perhaps on an I2C extender, code TBA!!!) to switch power.
- A GPIO pin to switch the signal direction
- A GPIO pin with PWM capability to switch the Brake (you may omit this if you dont want any DC capability)
- Optionally An Analog pin to read the current (unless your hardware cant do that, perhaps its just feeding a booster)
- Optionally a GPIO fault pin if thats how your hardware works. (NOT recommended as you're going to run out of pins)

IF you have no more than 3 tracks and you can arrange for the signal pins to be one of 11,12,13 on a Mega, THEN there is a slight advantage internally and the waveform will be super-sharp.

**Hardware that has two signal pins still needs some code thought!!!!!!!!**


## Configuring the Software

Configuring the software to provide more tracks is a simple extension of the existing method of customising the #define of MOTOR_SHIELD_TYPE in config.h
Since there can be no standard setup of your wiring and hardware choices, it will be necessary to create your custom built MOTOR_SHIELD_TYPE in the manner described in MotorDrivers.h and simply continue to add more `new MotorDriver(` definitions to the list, providing all the pin numbers and electronic limits for each track. (or even shorten the list to 1)

## Using EXRAIL to control Track Manager
EXRAIL has a single additional command that can be used to automate TM.

- `SET_TRACK(t,mode)`
where t is the track letter A..H and mode is one of
- `OFF`  track is switched off
- `DCC`  track gets DCC signal
- `PROG` track gets DCC prog signal
- `DC`   track is set to DC mode with the cab address of the currently executing EXRAIL sequence. 
- `DCX` as DC but with reversed polarity.

DC/DCX are designed so that you can be automating a DCC loco, drive it onto a separate track and switch to DC without having to know the cab address. (e.g AUTOMATION) 
If however you are just running a ROUTE you can always do something like this:
```
 ROUTE(77,"Set track G to DC 123")
   SETLOCO(123)
   SET_TRACK(G,DC)
   DONE
```

## Where and How for the Code.
The TM code is primarily in TrackManager.cpp which is responsible for coordinating the track settings and commands. 

Each individual track is handled by an instance of MotorDriver created from the MOTOPR_SHIELD_TYPE definition in config.h 

Many functions formerly in the DCCWaveform code have been moved to TrackManager or MotorDriver, notably the power control and checking. This makes the code easier to follow.
