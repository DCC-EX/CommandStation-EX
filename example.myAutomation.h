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
SET_TRACK(A,MAIN)
SET_TRACK(B,PROG)
POWERON        // turn on track power
SENDLOCO(3,1) // send loco 3 off along route 1
SENDLOCO(10,2) // send loco 10 off along route 2
DONE     // This just ends the startup thread, leaving 2 others running.

/* SEQUENCE(1) is a simple shuttle between 2 sensors      
 *  S167 and S168 are sensors on vpins 167 and 168 
 *  S167                   S168                   
 *  === START->================
 */
   SEQUENCE(1) 
     DELAY(10000)   // wait 10 seconds
     FON(3)       // Set Loco Function 3, Horn on
     DELAY(1000)    // wait 1 second
     FOFF(3)      // Horn off
     FWD(80)      // Move forward at speed 80
     AT(168)       // until we hit sensor 168
     STOP         // then stop
     DELAY(5000)    // Wait 5 seconds
     FON(2)       // ring bell
     REV(60)      // reverse at speed 60
     AT(167)       // until we get to S167
     STOP         // then stop
     FOFF(2)      // Bell off 
     FOLLOW(1)    // and follow sequence 1 again
   
/* SEQUENCE(2) is an automation example for a single loco Y shaped journey
 *  S164,S165,S166 are sensors, T4 is a turnout
 *  
 *  S166                      T4                        S164
 *  ===-START->=============================================
 *                          //
 *  S165                    //
 *  ======================//
 *  
 *  Train runs from START to S164, back to S165, again to S164, Back to start.
 */
  SEQUENCE(2)
   FWD(60)     // go forward at DCC speed 60 
   AT(164) STOP  // when we get to sensor 164 
   DELAY(10000)  // wait 10 seconds 
   THROW(4)    // throw turnout for route to S165
   REV(45)     // go backwards at speed 45
   AT(165) STOP  // until we arrive at sensor 165
   DELAY(5000)   // wait 5 seconds
   FWD(50)     // go forwards at speed 50
   AT(164) STOP  // and stop at sensor 164
   DELAY(5000)   // wait 5 seconds 
   CLOSE(4)    // set turnout closed
   REV(50)     // reverse back to S166
   AT(166) STOP
   DELAY(20000)  // wait 20 seconds 
   FOLLOW(2)   // follow sequence 2... ie repeat the process
