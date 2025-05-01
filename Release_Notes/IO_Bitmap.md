Virtual Bitmap device pins.

a Bitmap device pin is a software representation of a virtual hardware device that has the ability to store a 16bit value.

This this is easier to manage than LATCH in EXRAIL as they can be explicitely set and tested without interfering with underlying hardware or breaching the 255 limit.

Virtual pins may be set, reset and tested in the same way as any other pin. Unlike sensors and leds, these device pins are both INPUT and OUTPUT  These can be used in many ways:

  As a simple digital flag to assist in inter-thread communication.
  A flag or value that can be set from commands and tested in EXRAIL.(e.g. to stop a sequence)
  As a counter for looping or occupancy counts such as trains passing over a multi track road crossing.
  As a collection of 16 digital bits that can be set, reset, toggled, masked and tested.
  
  Existing <> and exrail commands for vpins work on these pins.  

  Virtual pin creation:
    HAL(Bitmap,firstpin,npins) 
       creates 1 or more virtual pins in software. (RAM requirement approximately 2 bytes per pin)
       e.g. HAL(Bitmap,1000,20)  creates pins 1000..1019

  Simple use as flags:
    This uses the traditional digital pin commands
       SET(1013) RESET(1013)  sets value 1 or 0
       SET(1000,20) RESET(1000,20)  sets/resets a range of pins  
       IF(1000) tests if pin value!=0

       Commands can set 1/0 values using <z 1010> <z -1010> as for any digital output.
       BLINK can be used to set them on/off on a time pattern. 

       In addition, Exrail sensor comands work as if these pins were sensors
          ONBUTTON(1013) triggers when value changes from 0 to something.
          ONSENSOR(1013) triggers when value changes to or from 0.
          <S 1013 1013 1> and JMRI_SENSOR(1013) report <Q/q responses when changing to or from 0.

    Use as analog values:
          Analog values may be set into the virtual pins and tested using the existing analog value commands and exrail macros.
          <z vpin value>  <D ANIN vpin> etc.

    Use as counters:
        For loop counting, counters can be incremented by BITMAP_INC(1013) and decremented by BITMAP_DEC(1013) and tested with IF/IFNOT/IFGTE etc.
        Counters be used to automate a multi track crossing where each train entering increments the counter and decrements it on clearing the crossing. Crossing gate automation can be started when the value changes from 0, and be stopped when the counter returns to 0.  Detecting the first increment from 0 to 1 can be done with ONBUTTON(1013) and the automation can use IF(1013) or IFNOT(1013) to detect when it needs to reopen the road gates. 

Use as binary flag groups: 
    Virtual pins (and others that respond to an analog read in order to provide bitmapped digital data, such as SensorCam) can be set and tested with new special EXRAIL commands

    IFBITMAP_ALL(vpin,mask)   Bitwise ANDs the  the vpin value with the mask value and is true if ALL the 1 bits in the mask are also 1 bits in the value. 
    e.g.    IFBITMAP_ALL(1013,0x0f)  would be true if ALL the last 4 bits of the value are 1s.
 
    IFBITMAP_ANY(1013,0x0f) would be true if ANY of the last 4 bits are 1s.


     Modifying bitmap values:
      BITMAP_AND(vpin,mask) performs a bitwise AND operation.
      BITMAP_OR(vpin,mask)  performa a bitwise OR operation
      BITMAP_XOR(vpin,mask) performs a bitwise EXCLUSIVE OR (which is basically a toggle)   


                      