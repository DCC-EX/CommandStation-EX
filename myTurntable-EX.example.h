/**************************************************************************************************
 * This is an example automation file to control Turntable-EX using recommended techniques.
 ************************************************************************************************** 
 * INSTRUCTIONS
 ************************************************************************************************** 
 * To use this example file as the starting point for your layout, there are two options:
 * 
 * 1. If you don't have an existing myAutomation.h file, simply rename this file.
 * 2. If you have an existing myAutomation.h file, rename this file to "myTurntable.h", and then
 *    include it by adding the line below at the end of your existing myAutomation.h, on a line of its own:
 * 
 *    #include "myTurntable.h"
 *************************************************************************************************/

/**************************************************************************************************
 * The MOVETT() command below will automatically move your turntable to the defined step position on
 * start up.
 * 
 * If you do not wish this to occur, simply comment the line out.
 * 
 * NOTE: If you are including this file at the end of an existing myAutomation.h file, you will likely
 * need to move this line to the beginning of your existing myAutomation.h file in order for it to
 * be effective.
 *************************************************************************************************/
MOVETT(600, 56, Turn)
DONE

// For Conductor level users who wish to just use Turntable-EX, you don't need to understand this
// and can move to defining the turntable positions below. You must, however, ensure this remains
// before any position definitions or you will get compile errors when uploading.
//
// Definition of the TURNTABLE_EX macro to correctly create the ROUTEs required for each position.
// This includes RESERVE()/FREE() to protect any automation activities.
//
#define TURNTABLE_EX(route_id, reserve_id, vpin, steps, activity, desc) \
  ROUTE(route_id, desc) \
    RESERVE(reserve_id) \
    MOVETT(vpin, steps, activity) \
    WAITFOR(vpin) \
    FREE(reserve_id) \
    DONE

/**************************************************************************************************
 * TURNTABLE POSITION DEFINITIONS
 *************************************************************************************************/
// TURNTABLE_EX(route_id, reserve_id, vpin, steps, activity, desc)
//
// route_id = A unique number for each defined route, the route is what appears in throttles
// reserve_id = A unique reservation number (0 - 255) to ensure nothing interferes with automation
// vpin = The Vpin defined for the Turntable-EX device driver, default is 600
// steps = The target step position
// activity = The activity performed for this ROUTE (Note do not enclose in quotes "")
// desc = Description that will appear in throttles (Must use quotes "")
//
TURNTABLE_EX(TTRoute1, Turntable, 600, 56, Turn, "Position 1")
TURNTABLE_EX(TTRoute2, Turntable, 600, 111, Turn, "Position 2")
TURNTABLE_EX(TTRoute3, Turntable, 600, 167, Turn, "Position 3")
TURNTABLE_EX(TTRoute4, Turntable, 600, 1056, Turn_PInvert, "Position 4")
TURNTABLE_EX(TTRoute5, Turntable, 600, 1111, Turn_PInvert, "Position 5")
TURNTABLE_EX(TTRoute6, Turntable, 600, 1167, Turn_PInvert, "Position 6")
TURNTABLE_EX(TTRoute7, Turntable, 600, 0, Home, "Home turntable")

// Pre-defined aliases to ensure unique IDs are used.
// Turntable reserve ID, valid is 0 - 255
ALIAS(Turntable, 255)

// Turntable ROUTE ID reservations, using <? TTRouteX> for uniqueness:
ALIAS(TTRoute1, 5179)
ALIAS(TTRoute2, 5180)
ALIAS(TTRoute3, 5181)
ALIAS(TTRoute4, 5182)
ALIAS(TTRoute5, 5183)
ALIAS(TTRoute6, 5184)
ALIAS(TTRoute7, 5185)
ALIAS(TTRoute8, 5186)
ALIAS(TTRoute9, 5187)
ALIAS(TTRoute10, -13746)
ALIAS(TTRoute11, -13745)
ALIAS(TTRoute12, -13744)
ALIAS(TTRoute13, -13743)
ALIAS(TTRoute14, -13742)
ALIAS(TTRoute15, -13741)
ALIAS(TTRoute16, -13740)
ALIAS(TTRoute17, -13739)
ALIAS(TTRoute18, -13738)
ALIAS(TTRoute19, -13737)
ALIAS(TTRoute20, -13736)
ALIAS(TTRoute21, -13735)
ALIAS(TTRoute22, -13734)
ALIAS(TTRoute23, -13733)
ALIAS(TTRoute24, -13732)
ALIAS(TTRoute25, -13731)
ALIAS(TTRoute26, -13730)
ALIAS(TTRoute27, -13729)
ALIAS(TTRoute28, -13728)
ALIAS(TTRoute29, -13727)
ALIAS(TTRoute30, -13726)