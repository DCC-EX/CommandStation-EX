Railcom implementation notes, Chris Harlow Oct 2024

Railcom support is in 3 parts
1. Generation of the DCC waveform with a Railcom cutout.
2. Accessing the railcom feedback from a loco using hardware detectors
3. Utilising the feedback to do something useful.

DCC Waveform Railcom cutout depends on using suitable motor shields (EX8874 primarily) as the standard Arduino shield is not suitable. (Too high resistance during cutout)
The choice of track management also depends on wiring all the MAIN tracks to use the same signal and brake pins. This allows separate track power management but prevents switching a single track from MAIN to PROG or DC... 
Some CPUs require very specific choice of brake pins etc to match their internal timer register architecture.

- MEGA.. The default shield setting for an EX8874 is suitable for Railcom on Channel A (MAIN) 
- ESP32 .. not yet supported.
- Nucleo ... TBA 

Enabling the Railcom Cutout requires a `<C RAILCOM ON>` command. This can be added to myAutomation using `PARSE("<C RAILCOM ON>")`
Code to calculate the cutout position and provide synchronization for the sampling is in `DCCWaveform.cpp` (not ESP32)
and in general a global search for "railcom" will show all code changes that have been made to support this.

Code to actually implement the timing of the cutout is highly cpu dependent and can be found in the various implementations of `DCCTimer.h`. At this time only `DCCTimerAVR.cpp`has implemented this.  


Reading Railcom data:
  A new HAL handler (`IO_I2CRailcom.h`) has been added to process input from a 32-block railcom collecter which operates over I2C. The collector and its readers  sit between the CS and the track and collect railcom data from locos during the cutout.
  The Collector device removes 99.9% of the railcom traffic and returns just a summary of what has changed since the last cutout.
  After the cutout the HAL driver reads the Collector summary over I2C and passes the raw data to the CS logic (`Railcom.cpp`) for analysis.

  Each 32-block reader is described in myAutomation like `HAL(I2CRailcom,10000,32,0x08)`  which will assign 32 blocks on i2c address 0x08 with vpin numbers 10000 and 10031. If you only use fewer channel in the collector, you can assign fewer pins here.
  (Implementation notes.. you may have multiple collectors, each one will requite a HAL line to define its i2c address and vpins to represent block numbers.)

Making use of Railcom data

 Exrail has two additional event handlers which can capture locos entering and exiting blocks. These handlers are started with the loco information already set, so for example:
 ```
 ONBLOCKENTER(10000) 
    // a loco has entered block 10000 
    FON(0)  // turn the light on
    FON(1)  // make a lot of noise
    SPEED(20) // slow down
    DONE 
 
 ONBLOCKEXIT(10000) 
    // a loco has left block 10000 
    FOFF(0)  // turn the light off
    FOFF(1)  // stop the noise
    SPEED(50) // speed up again
    DONE 
    ```

    Note that the Railcom interpretation code is capable of detecting multiple locos in the same block at the same time and will create separate exrail tasks for each one.
     There is however one minor loophole in the block exit logic...
      If THREE or more locos are in the same block and ONE of them leaves, then ONBLOCKEXIT will not fire until 
      EITHER - The leaving loco enters another railcom block
      OR     - only ONE loco remains in the block just left.

      To further support block management in railcom, two additional serial commands are available

      `<K block loco >` to simulate a loco entering a block, and trigger any ONBLOCKENTER
      `<k block loco >` to simulate a loco leaving a block, and trigger and ONBLOCKEXIT


   Reading CV values on MAIN.

      Railcom allows for the facility to read loco cv values while on the main track. This is considerably faster than PROG track access but depends on the loco being in a Railcom monitored block. 

      To read from PROG Track we use `<R cv>` response is `<r value>` 

      To read from MAIN track use `<r loco cv>`
        response is `<r loco cv value>`  

     