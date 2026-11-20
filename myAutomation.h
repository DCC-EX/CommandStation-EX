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

//IR Pin Definitions:
//Mega: 23, 25
//GPIO 1: 164, 165
//GPIO 2: 181, 182

 
AUTOMATION(1, "Sensor Stop Test")
AUTOSTART
POWERON
SETLOCO(3)
FWD(25)
DELAY(1000)
FOLLOW(2)

SEQUENCE(2)
SPEED(60)
AT(23)          // wait for the train to arrive at the sensor
SPEED(30)
DELAY(2000)
STOP
DELAY(3000)
INVERT_DIRECTION
SPEED(30)       // leave the station slowly
AT(-23)         // wait until the sensor goes inactive (train has left)
DELAY(1000)     // short buffer so it's fully clear
FOLLOW(2)
