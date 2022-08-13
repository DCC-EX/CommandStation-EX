/**************************************************************************************************
 * This is an example automation file to control EX-Turntable using recommended techniques.
 ************************************************************************************************** 
 * INSTRUCTIONS
 ************************************************************************************************** 
 * To use this example file as the starting point for your layout, there are two options:
 * 
 * 1. If you don't have an existing "myAutomation.h" file, simply rename "myEX-Turntable.example.h" to
 *    "myAutomation.h".
 * 2. If you have an existing "myAutomation.h" file, rename "myEX-Turntable.example.h" to "myEX-Turntable.h",
 *    and then include it by adding the line below at the end of your existing "myAutomation.h", on a
 *    line of its own:
 * 
 *    #include "myEX-Turntable.h"
 * 
 * Note that there are further instructions in the documentation at https://dcc-ex.com/.
 *************************************************************************************************/

/**************************************************************************************************
 * The MOVETT() command below will automatically move your turntable to the defined step position on
 * start up.
 * 
 * If you do not wish this to occur, simply comment the line out.
 * 
 * NOTE: If you are including this file at the end of an existing "myAutomation.h" file, you will likely
 * need to move this line to the beginning of your existing "myAutomation.h" file in order for it to
 * be effective.
 *************************************************************************************************/
MOVETT(600, 114, Turn)
DONE

// For Conductor level users who wish to just use EX-Turntable, you don't need to understand this
// and can move to defining the turntable positions below. You must, however, ensure this remains
// before any position definitions or you will get compile errors when uploading.
//
// Definition of the EX_TURNTABLE macro to correctly create the ROUTEs required for each position.
// This includes RESERVE()/FREE() to protect any automation activities.
//
#define EX_TURNTABLE(route_id, reserve_id, vpin, steps, activity, desc) \
  ROUTE(route_id, desc) \
    RESERVE(reserve_id) \
    MOVETT(vpin, steps, activity) \
    WAITFOR(vpin) \
    FREE(reserve_id) \
    DONE

/**************************************************************************************************
 * TURNTABLE POSITION DEFINITIONS
 *************************************************************************************************/
// EX_TURNTABLE(route_id, reserve_id, vpin, steps, activity, desc)
//
// route_id = A unique number for each defined route, the route is what appears in throttles
// reserve_id = A unique reservation number (0 - 255) to ensure nothing interferes with automation
// vpin = The Vpin defined for the Turntable-EX device driver, default is 600
// steps = The target step position
// activity = The activity performed for this ROUTE (Note do not enclose in quotes "")
// desc = Description that will appear in throttles (Must use quotes "")
//
EX_TURNTABLE(TTRoute1, Turntable, 600, 114, Turn, "Position 1")
EX_TURNTABLE(TTRoute2, Turntable, 600, 227, Turn, "Position 2")
EX_TURNTABLE(TTRoute3, Turntable, 600, 341, Turn, "Position 3")
EX_TURNTABLE(TTRoute4, Turntable, 600, 2159, Turn, "Position 4")
EX_TURNTABLE(TTRoute5, Turntable, 600, 2273, Turn, "Position 5")
EX_TURNTABLE(TTRoute6, Turntable, 600, 2386, Turn, "Position 6")
EX_TURNTABLE(TTRoute7, Turntable, 600, 0, Home, "Home turntable")

// Pre-defined aliases to ensure unique IDs are used.
// Turntable reserve ID, valid is 0 - 255
ALIAS(Turntable, 255)

// Turntable ROUTE ID reservations, using <? TTRouteX> for uniqueness:
ALIAS(TTRoute1)
ALIAS(TTRoute2)
ALIAS(TTRoute3)
ALIAS(TTRoute4)
ALIAS(TTRoute5)
ALIAS(TTRoute6)
ALIAS(TTRoute7)
ALIAS(TTRoute8)
ALIAS(TTRoute9)
ALIAS(TTRoute10)
ALIAS(TTRoute11)
ALIAS(TTRoute12)
ALIAS(TTRoute13)
ALIAS(TTRoute14)
ALIAS(TTRoute15)
ALIAS(TTRoute16)
ALIAS(TTRoute17)
ALIAS(TTRoute18)
ALIAS(TTRoute19)
ALIAS(TTRoute20)
ALIAS(TTRoute21)
ALIAS(TTRoute22)
ALIAS(TTRoute23)
ALIAS(TTRoute24)
ALIAS(TTRoute25)
ALIAS(TTRoute26)
ALIAS(TTRoute27)
ALIAS(TTRoute28)
ALIAS(TTRoute29)
ALIAS(TTRoute30)
