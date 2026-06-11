# Creating a Mimic Panel node

A mimic panel is a great example of node which can operate entirely independently of the command station or other nodes.

Typically a Mimic panel would need to:

- Show turnout states
- Show signal states
- Show track occupied
- Have buttons to toggle tuirnouts
- Display texts on an Oled screen

All this can be built into a CS but it is easier to build this as a node which can stand alone and handle all the above without any change to the existing CS or node logic.

## Local accessories

Typically the Mimic panel may use a neopixel strip to provide lighting and  buttons to brovide control. All these can be locally attached to the node and remain invisible to the remainder of the network. The HAL setup and commands are exactly as done on the Command Station so there is nothing extra to learn. 

## Monitoring turnouts

By adding the REMOTE_TURNOUT definition to the mimic myAutomation, the exrail script can monitor the turnout state with ONTHROW etc and use that to set pixels on the panel.

e.g.

```cpp
REMOTE_TURNOUT(22)
ONTHROW(22) SET(622) RESET(522) DONE
ONCLOSE(22) SET(522) RESET(622) DONE
```

Note that it does not care where the actual turnout 22 is located.

Similarly, the script may detect a button press on the mimic panel and use that to toggle a tuirnout

```cpp
ONBUTTON(722) TOGGLE_TURNOUT(22) DONE
```

In this way, the layout operates normally whether the mimic panel is there or not. And because there is no concept of a node id, you could build a second mimic panel with exactly the same software build and have multiple identical panels at either end of your magnificent layout.
