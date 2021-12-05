
/* This is an automation example file.
 *  The presence of a file calle "myAutomation.h" brings EX-RAIL code into
 *  the command station.
 *  The auotomation may have multiple concurrent tasks. 
 *  A task may drive one loco through a ROUTE or may simply 
 *  automate some other part of the layout without any loco.
 *  
 *  At startup, a single task is created to execute the first 
 *  instruction after ROUTES. 
 *  This task may simply follow a route, or may SCHEDULE  
 *  further tasks (thats is.. send a loco out along a route).
 *  
 *  Where the loco id is not known at compile time, a new task 
 *  can be creatd with the command:
 *  </ SCHEDULE [cab] route> 
 *  
 */

// Include the name to pin mappings for my layout 
#include "myLayout.h"

ALIAS(ROUTE_1,1)
ALIAS(UP_MOUNTAIN,8)
ALIAS(UP_MOUNTAIN_FROM_PROG,88)
ALIAS(INNER_LOOP,7) 
ALIAS(INNER_FROM_PROG,77) 

//EXRAIL   // myAutomation must start with the EXRAIL instruction
  // This is the default starting route, AKA ROUTE(0) 
 // START(999) // this is just a diagnostic test cycle 
  PRINT("started")
  LCD(0,"EXRAIL RULES")
  SERIAL("I had one of them but the leg fell off!")
  DONE     // This just ends the startup thread

  
  /*AUTOSTART*/ ROUTE(ROUTE_1,"Close All") 
    LCD(1,"Bingo")
    CLOSE(TOP_TURNOUT) DELAY(10)
    CLOSE(Y_TURNOUT) DELAY(10)
    CLOSE(MIDDLE_TURNOUT) DELAY(10)
    CLOSE(JOIN_TURNOUT) DELAY(10)
    CLOSE(LOWER_TURNOUT) DELAY(10)
    CLOSE(CROSSOVER_TURNOUT) DELAY(10)
    CLOSE(PROG_TURNOUT) DELAY(10)
    PRINT("Close All completed")

    ENDTASK
    

   SEQUENCE(UP_MOUNTAIN)  // starting at the lower closed turnout siding and going up the mountain 
     PRINT("Up Mountain started")
     DELAY(10000)   // wait 10 seconds
     RESERVE(BLOCK_LOWER_MOUNTAIN)
     CLOSE(LOWER_TURNOUT) CLOSE(JOIN_TURNOUT)
     FWD(60) AT(Y_LOWER)
     RESERVE(BLOCK_X_MOUNTAIN)
    CLOSE(Y_TURNOUT) CLOSE(MIDDLE_TURNOUT)
    FWD(40)  AT(MIDDLE_C_BUFFER) STOP
    FREE(BLOCK_X_MOUNTAIN) FREE(BLOCK_LOWER_MOUNTAIN)
    DELAY(10000)
    RESERVE(BLOCK_UPPER_MOUNTAIN) RESERVE(BLOCK_X_MOUNTAIN)
    CLOSE(MIDDLE_TURNOUT) THROW(Y_TURNOUT) THROW(TOP_TURNOUT)
    REV(55)
    AFTER(Y_UPPER) FREE(BLOCK_X_MOUNTAIN)
    REV(55) AT(TOP_T_BUFFER) STOP  // At top of mountain
    FREE(BLOCK_UPPER_MOUNTAIN)
    DELAY(5000)
    RESERVE(BLOCK_UPPER_MOUNTAIN)
    THROW(TOP_TURNOUT)
    FWD(60) AT(Y_UPPER)
    RESERVE(BLOCK_X_MOUNTAIN)
    THROW(Y_TURNOUT) CLOSE(MIDDLE_TURNOUT)
     FWD(40) AT(MIDDLE_C_BUFFER) STOP
     FREE(BLOCK_UPPER_MOUNTAIN) FREE(BLOCK_X_MOUNTAIN)
     DELAY(6000)
     RESERVE(BLOCK_LOWER_MOUNTAIN) RESERVE(BLOCK_X_MOUNTAIN)
     CLOSE(MIDDLE_TURNOUT) CLOSE(Y_TURNOUT) CLOSE(JOIN_TURNOUT) CLOSE(LOWER_TURNOUT)
     REV(60) 
     AFTER(Y_LOWER) FREE(BLOCK_X_MOUNTAIN)
     AT(LOWER_C_BUFFER) STOP
     FREE(BLOCK_LOWER_MOUNTAIN)
     FOLLOW(UP_MOUNTAIN) 

AUTOMATION(UP_MOUNTAIN_FROM_PROG,"Send up mountain from prog")
   JOIN
   RESERVE(BLOCK_LOWER_MOUNTAIN)
   RESERVE(BLOCK_X_INNER)
   RESERVE(BLOCK_X_OUTER)
   // safe to cross
   THROW(PROG_TURNOUT) THROW(CROSSOVER_TURNOUT) THROW(JOIN_TURNOUT)
   FWD(45)
   AFTER(JOIN_AFTER) STOP
   CLOSE(PROG_TURNOUT) CLOSE(CROSSOVER_TURNOUT) CLOSE(JOIN_TURNOUT)
   FREE(BLOCK_X_OUTER) FREE(BLOCK_X_INNER)
   CLOSE(LOWER_TURNOUT)
   REV(40) AT(LOWER_C_BUFFER) STOP
   FREE(BLOCK_LOWER_MOUNTAIN)   
   FOLLOW(UP_MOUNTAIN)

   SEQUENCE(INNER_LOOP) 
     FWD(50)
     AT(CROSSOVER_INNER_BEFORE)
     RESERVE(BLOCK_X_INNER)
     CLOSE(CROSSOVER_TURNOUT)
     FWD(50)
     AFTER(CROSSOVER_INNER_AFTER)
     FREE(BLOCK_X_INNER)
     FOLLOW(INNER_LOOP)

     
   // Turnout definitions
   TURNOUT(TOP_TURNOUT, TOP_TURNOUT,0,"Top Station")
   TURNOUT(Y_TURNOUT, Y_TURNOUT,0,"Mountain join")
   TURNOUT(MIDDLE_TURNOUT, MIDDLE_TURNOUT,0,"Middle Station")
   TURNOUT(JOIN_TURNOUT,JOIN_TURNOUT,0)
   TURNOUT(LOWER_TURNOUT,LOWER_TURNOUT,0)
   TURNOUT(CROSSOVER_TURNOUT,CROSSOVER_TURNOUT,0)
   TURNOUT(PROG_TURNOUT,PROG_TURNOUT,0)
   
// Single slip protection
   ONTHROW(2)
     THROW(1)
     DONE
   ONCLOSE(1)
     CLOSE(2)
     DONE


   ROUTE(61,"Call return test")
   PRINT("In 61 test 1")
   CALL(62)
   PRINT("In 61 test 2")
   CALL(62)
   PRINT("In 61 test 3")
   ACTIVATE(100,2)
   DEACTIVATE(100,2) 
   DONE

   SEQUENCE(62)
     PRINT("In seq 62")
     RETURN

  ROUTE(63,"Signal test 40,41,42")
   SIGNAL(40,41,42)
   DELAY(2000)
   RED(40)
   DELAY(2000)
   AMBER(40)
   DELAY(2000)
   GREEN(40)
   FOLLOW(63)


   ROUTE(64,"Func test 6772")
   XFON(6772,1)
   DELAY(5000)
   XFOFF(6772,1)
   DELAY(5000)
   FOLLOW(64)

ROUTE(65,"Negative sensor test")
  PRINT(" WAIT for -176")
  AT(-176)
  PRINT(" WAIT for 176")
  AT(176)
  PRINT("done")
  DONE 

ROUTE(123,"Activate stuff")
  ACTIVATEL(5)
  ACTIVATE(7,2)
  DEACTIVATE(3,2)
  DEACTIVATEL(6) 
  DONE

ONACTIVATEL(5)
   PRINT("ACT 5")
   DONE
ONACTIVATE(7,2)
   PRINT("ACT 7,2")
   DONE
ONDEACTIVATE(7,2)
   PRINT("DEACT 7,2")
   DONE
ONDEACTIVATEL(5)
   PRINT("DEACT 5")
   DONE
   
    
   
