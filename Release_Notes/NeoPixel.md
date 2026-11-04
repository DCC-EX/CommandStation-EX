# NeoPixel support

The IO_NeoPixel.h driver supports the adafruit neopixel seesaw board. It turns each pixel into an individual VPIN which can be given a colour and turned on or off using the new <o> command or the NEOPIXEL Exrail macro. Exrail SIGNALS can also drive a single pixel signal or multiple separate pixels.

For a directly connected ESP32 pixel string, use the RMT driver instead of a Seesaw board:

`  HAL(NeoPixelRMT,1000,20,18,NEO_GRB)`

The parameters are the first vpin, pixel count, ESP32 data GPIO and optional colour/speed mode.
With DCC RMT enabled, this driver reserves channels starting at channel 4, after the channels used by the main and programming tracks. This shared use is supported on the classic ESP32; other ESP32 variants may not have free RMT transmit channels.

For the ESP32 RMT driver, `<o firstVpin CHASE count brightness>` runs the
red/green/blue test once over the selected pixel range and then clears it.
Brightness is 0 to 255; for example, `<o 1000 CHASE 20 64>` starts at VPIN
1000 and chases across 20 pixels at brightness 64. The selected VPIN range
must belong to one configured `HAL(NeoPixelRMT,...)` strip.
`<o firstVpin FLASH count brightness>` repeatedly displays white, red, green,
blue, and off across the selected range, changing color once per second. For
example, `<o 1000 FLASH 20 64>` flashes 20 pixels at brightness 64. The range
must belong to one configured `HAL(NeoPixelRMT,...)` strip.


1.  Defining the hardware driver:

    Add a driver definition in myAutomation.h for each adafruit I2C driver.

    `HAL(neoPixel, firstVpin, numberOfPixels [, mode [, i2caddress])`
      Where mode is selected from the various pixel string types which have varying
      colour order or refresh frequency. For MOST strings this mode will be NEO_GRB but for others refer to the comments in IO_NeoPixel.h
      If omitted the node and i2caddress default to NEO_GRB, 0x60.

    `HAL(NeoPixel,1000,20)`
       This is a NeoPixel driver defaulting to I2C aqddress 0x60 for a GRB pixel string. Pixels are given vpin numbers from 1000 to 1019. 
    `HAL(NeoPixel,1020,20,NEO_GRB,0x61)`
       This is a NeoPixel driver on i2c address 0x61

2. Setting pixels from the `< >` commands.
    By default, each pixel in the string is created as white but switched off.
    Each pixel has a vpin starting from the first vpin in the HAL definitions.

    `<o vpin>`   switches pixel on  (same as `<z vpin>`) e.g. `<o 1005>`
    `<o -vpin>` switches pixel off (same as `<z -vpin>`) e.g. `<o -1003>`
    (the z commands work on pixels the same as other gpio pins.)

    `<o [-]vpin count>` switches on/off count pixels starting at vpin. e.g `<o 1000 5>`
    Note: it IS acceptable to switch across 2 strings of pixels if they are contiguous vpin ranges.  It is also interesting that this command doesnt care if the vpins are NeoPixel or any other type, so it can be used to switch a range of other pin types. 

    `<o [-]vpin red green blue [count [brightness]]>` sets the colour and on/off status of a pin or pins. Each colour and brightness is 0..255. Brightness defaults to 255. For example, `<o 1005 255 255 0>` sets pin 1005 to full-brightness yellow; `<o 1000 255 0 0 20 64>` sets 20 pixels to dim red. To set brightness for one pixel, specify a count of 1 first.
    `<o firstVpin CHASE count brightness>` runs the ESP32 RMT NeoPixel chase over the selected range.
    `<o firstVpin FLASH count brightness>` cycles the ESP32 RMT NeoPixel range through white, red, green, blue, and off, changing every second.
    Note: If you set a pin to a colour, you can turn it on and off without having to reset the colour every time. This is something the adafruit seesaw library can't do and is just one of several reasons why we dont use it.  

    To randomly show and hide the NeoPixels configured at VPINs 1000 and 2000,
    run `python3 scripts/random_neopixels.py /dev/ttyUSB0`. The script sends
    500 random `<o [-]vpin red green blue count brightness>` commands by default,
    varying RGB color, brightness, on/off state, and LED count (1 to 5), with
    short random delays. It prints a startup-wait message before the configured
    startup delay.
    Install its serial dependency with `apt-get install python3-serial` on linux
    or on windows `python3 -m pip install pyserial`.
    Use `--count`, `--baud`, `--min-delay`, and `--max-delay` to adjust the run;
    pass `--help` for all options.

3. Setting pixels from EXRAIL
    The NEOPIXEL macro sets an RGB colour, with an optional count:
    `NEOPIXEL([-]vpin, red, green, blue [,count])`
    To set brightness, BRIGHTNESS scales the RGB values before they are encoded:
    `NEOPIXEL(700, BRIGHTNESS(90,255,50,77), 8)`
    BRIGHTNESS takes a brightness level from 0 to 255 followed by red, green, and blue components from 0 to 255. Brightness values outside the range are clamped.

    To change brightness without supplying RGB again, use:
    `NEOPIXEL_BRIGHTNESS(vpin, brightness [,count])`
    Count defaults to 1: `NEOPIXEL_BRIGHTNESS(1000, 64)` dims a single pixel.
    For example, `NEOPIXEL_BRIGHTNESS(1000, 64, 8)` dims eight consecutive pixels.
    Brightness is an absolute scale from 0 (dark) to 255 (full), clamped to this
    range. It preserves each pixel's stored RGB colour and on/off state, including
    blinking. Use 255 to restore the original colour intensity, even after 0.
    The scale remains in effect when a new colour is set with NEOPIXEL.
    Supported by the NeoPixel I2C and ESP32 NeoPixelRMT drivers, including shared
    VPINs on nodes running firmware with this support.

    Setting pixels on or off (without colour change) can be done with SET/RESET [currently there is no set range facility but that may be added as a general exrail thing... watch this space]

    Because the pixels obey set/reset, the BLINK command can also be used to control blinking a pixel.

4. EXRAIL pixel signals.
    There are two types possible, a mast with separate fixed colour pixels for each aspect, or a mast with one multiple colour pixel for all aspects.

    For separate pixels, the colours should be established at startup and a normal SIGNALH macro used.
    
    ```
    AUTOSTART 
        SIGNALH(1010,1011,1012)
        NEOPIXEL(1010,255,0,0)       
        NEOPIXEL(1011,128,128,0)
        NEOPIXEL(1012,0,255,0)
        RED(1010)  // force signal state otherwise all 3 lights will be on
        DONE 
    ```
    For signals with 1 pixel, the NEOPIXEL_SIGNAL macro will create a signal
    `NEOPIXEL_SIGNAL(vpin,redfx,amberfx,greenfx)`

    The fx values above can be created by the NeoRGB macro: bright red is
    `NeoRGB(255,0,0)`, bright green is `NeoRGB(0,255,0)`, and amber can be
    `NeoRGB(255,100,0)`. To dim a colour, wrap it in `BRIGHTNESS(level,red,green,blue)`
    inside `NeoRGB`; brightness and colour components range from 0 to 255.
    `NeoRGB` creates one packed colour value for use in signal definitions.
    ```
    // create 1-lamp signal with NeoRGB colours
    NEOPIXEL_SIGNAL(1000,
        NeoRGB(255,0,0),
        NeoRGB(255,100,0),
        NeoRGB(0,255,0))

    // Create a dimmer signal by setting a brightness level per aspect
    NEOPIXEL_SIGNAL(1003,
        NeoRGB(BRIGHTNESS(90,255,0,0)),
        NeoRGB(BRIGHTNESS(90,255,100,0)),
        NeoRGB(BRIGHTNESS(90,0,255,0)))

    // Create 1-lamp signal with named colours.
    // This is better if you have multiple signals.
    // (Note: ALIAS is not suitable due to word length defaults) 
    #define REDLAMP NeoRGB(255,0,0)
    #define AMBERLAMP NeoRGB(255,100,0)
    #define GREENLAMP NeoRGB(0,255,0)
    NEOPIXEL_SIGNAL(1001,REDLAMP,AMBERLAMP,GREENLAMP)

    // Create 1-lamp signal with web type RGB colours 
    // (Using blue for the amber  signal , just testing) 
    NEOPIXEL_SIGNAL(1002,0xFF0000,0x0000FF,0x00FF00)
    ```
