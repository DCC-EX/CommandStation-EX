/* This is an automation example file.
 *  The presence of a file called "myAutomation.h" brings EX-RAIL code into
 *  the command station.
 *  The automation may have multiple concurrent tasks.
 *  A task may 
 *  - Act as a ROUTE setup macro for a user to drive over 
 *  - drive a loco through an AUTOMATION 
 *  - automate some cosmetic part of the layout without any loco.
 *  
 *  At startup, a single task is created to execute the startup sequence.
 *  This task may simply follow a route, or may START  
 *  further tasks (that is.. send a loco out along a route).
 *  
 *  Where the loco id is not known at compile time, a new task 
 *  can be created with the command:
 *  </ START [cab] route> 
 *  
 *  A ROUTE, AUTOMATION or SEQUENCE are internally identical in ExRail terms  
 *  but are just represented differently to a Withrottle user:
 *  ROUTE(n,"name") - as Route_n .. to setup a route through a layout
 *  AUTOMATION(n,"name") as Auto_n .. to send the current loco off along an automated journey
 *  SEQUENCE(n) is not visible to Withrottle.
 *  
 */

// This is the startup sequence, 
AUTOSTART
POWERON        // turn on track power
SENDLOCO(3,1) // send loco 3 off along route 1
SENDLOCO(10,2) // send loco 10 off along route 2
DONE     // This just ends the startup thread, leaving 2 others running.

/* SEQUENCE(1) is a simple shuttle between 2 sensors      
 *  S20 and S21 are sensors on arduino pins 20 and 21 
 *  S20                    S21                   
 *  === START->================
 */
   SEQUENCE(1) 
     DELAY(10000)   // wait 10 seconds
     FON(3)       // Set Loco Function 3, Horn on
     DELAY(1000)    // wait 1 second
     FOFF(3)      // Horn off
     FWD(80)      // Move forward at speed 80
     AT(21)       // until we hit sensor id 21
     STOP         // then stop
     DELAY(5000)    // Wait 5 seconds
     FON(2)       // ring bell
     REV(60)      // reverse at speed 60
     AT(20)       // until we get to S20
     STOP         // then stop
     FOFF(2)      // Bell off 
     FOLLOW(1)    // and follow sequence 1 again
   
/* SEQUENCE(2) is an automation example for a single loco Y shaped journey
 *  S31,S32,S33 are sensors, T4 is a turnout
 *  
 *  S33                      T4                            S31
 *  ===-START->=============================================
 *                          //
 *  S32                    //
 *  ======================//
 *  
 *  Train runs from START to S31, back to S32, again to S31, Back to start.
 */
  SEQUENCE(2)
   FWD(60)     // go forward at DCC speed 60 
   AT(31) STOP  // when we get to sensor 31 
   DELAY(10000)  // wait 10 seconds 
   THROW(4)    // throw turnout for route to S32
   REV(45)     // go backwards at speed 45
   AT(32) STOP  // until we arrive at sensor 32
   DELAY(5000)   // wait 5 seconds
   FWD(50)     // go forwards at speed 50
   AT(31) STOP  // and stop at sensor 31
   DELAY(5000)   // wait 5 seconds 
   CLOSE(4)    // set turnout closed
   REV(50)     // reverse back to S3
   AT(33) STOP
   DELAY(20000)  // wait 20 seconds 
   FOLLOW(2)   // follow sequence 2... ie repeat the process
