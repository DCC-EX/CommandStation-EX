NeoPixel support

The IO_NeoPixel.h driver supports the adafruit neopixel seesaw board. It turns each pixel into an individual VPIN which can be given a colour and turned on or off using the new <o> command or the NEOPIXEL Exrail macro. Exrail SIGNALS can also drive a single pixel signal or multiple separate pixels.


1.  Defining the hardware driver:
    Add a driver definition in myAutomation.h for each adafruit I2C driver.

    HAL(neoPixel, firstVpin, numberOfPixels [, mode [, i2caddress])
      Where mode is selected from the various pixel string types which have varying
      colour order or refresh frequency. For MOST strings this mode will be NEO_GRB but for others refer to the comments in IO_NeoPixel.h
      If omitted the node and i2caddress default to NEO_GRB, 0x60.

    HAL(NeoPixel,1000,20)
       This is a NeoPixel driver defaulting to I2C aqddress 0x60 for a GRB pixel string. Pixels are given vpin numbers from 1000 to 1019. 
    HAL(NeoPixel,1020,20,NEO_GRB,0x61)
       This is a NeoPixel driver on i2c address 0x61    

2. Setting pixels from the < > commands.
    By default, each pixel in the string is created as white but switched off.
    Each pixel has a vpin starting from the first vpin in the HAL definitions.

    <o vpin>   switches pixel on  (same as <z vpin>) e.g. <o 1005>
    <o -vpin> switches pixel off (same as <z -vpin>) e.g. <o -1003>
    (the z commands work on pixels the same as other gpio pins.)

    <o [-]vpin count> switches on/off count pixels starting at vpin. e.g <o 1000 5>
    Note: it IS acceptable to switch across 2 strings of pixels if they are contiguous vpin ranges.  It is also interesting that this command doesnt care if the vpins are NeoPixel or any other type, so it can be used to switch a range of other pin types. 

    <o [-]vpin red green blue [count]> sets the colour and on/off status of a pin or pins. Each colour is 0..255 e.g. <o 1005 255 255 0>  sets pin 1005 to bright yellow and ON, <0 -1006 0 0 255 10>  sets pins 1006 to 1015 (10 pins) to bright blue but OFF.     
    Note: If you set a pin to a colour, you can turn it on and off without having to reset the colour every time. This is something the adafruit seesaw library can't do and is just one of several reasons why we dont use it.  

3. Setting pixels from EXRAIL
    The new NEOPIXEL macro provides the same functionality as the    <o [-]vpin red green blue [count]> command above. 
    NEOPIXEL([-]vpin, red, green, blue [,count])
    
    Setting pixels on or off (without colour change) can be done with SET/RESET [currently there is no set range facility but that may be added as a general exrail thing... watch this space]

    Because the pixels obey set/reset, the BLINK command can also be used to control blinking a pixel.

4. EXRAIL pixel signals.
    There are two types possible, a mast with separate fixed colour pixels for each aspect, or a mast with one multiple colour pixel for all aspects.

    For separate pixels, the colours should be established at startup and a normal SIGNALH macro used.
    
  AUTOSTART 
    SIGNALH(1010,1011,1012)
    NEOPIXEL(1010,255,0,0)       
    NEOPIXEL(1011,128,128,0)
    NEOPIXEL(1012,0,255,0)
    RED(1010)  // force signal state otherwise all 3 lights will be on
    DONE 

    For signals with 1 pixel, the NEOPIXEL_SIGNAL macro will create a signal 
    NEOPIXEL_SIGNAL(vpin,redfx,amberfx,greenfx)

    ** Changed... ****
    The fx values above can be created by the NeoRGB macro so a bright red would be NeoRGB(255,0,0)  bright green NeoRGB(0,255,0) and amber something like NeoRGB(255,100,0)
    NeoRGB creates a single int32_t value so it can be used in several ways as convenient.

// create 1-lamp signal with NeoRGB colours
NEOPIXEL_SIGNAL(1000,NeoRGB(255,0,0),NeoRGB(255,100,0),NeoRGB(0,255,0))

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

      
  