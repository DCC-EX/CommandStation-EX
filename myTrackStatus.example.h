// ************* OLED JL Display Track mA Amperage  **************** //
// by Herb Morton    [Ash]                          March 23, 2025 
//    Colin Murdoch  [ColinM] 

//  Inside your config.h file First edit OLED max char rows set to 17
// #define MAX_CHARACTER_ROWS 17

// myAutomation.h
// Reporting power status and mA for each track on the LCD
AUTOSTART DELAY(5000) 
 ROUTE(238, "Resume/Pause JL Display")
  IF(236) 
    UNLATCH(236)
     ROUTE_CAPTION(238, "Paused") ROUTE_INACTIVE(238)
     PRINT("Pause JL Display")
      SCREEN(0, 8, "Track status paused")
      SCREEN(0, 9, "")
      SCREEN(0,10, "")   // several blank lines as needed
      SCREEN(0,11, "")
      SCREEN(0,12, "")
      SCREEN(0,13, "")
      SCREEN(0,14, "")
      SCREEN(0,15, "")
      SCREEN(0,16, "")
    DONE ENDIF
  LATCH(236) 
   ROUTE_CAPTION(238, "Running") ROUTE_ACTIVE(238)
    PRINT("Resume JL Display")
   FOLLOW(237)
  SEQUENCE(237)
   PARSE("<JL 0 8>")  // screen 0  start on line 8
    PRINT("\n")
    DELAY(3000)
  IF(236) FOLLOW(237) ENDIF
  DONE
// ************ End OLED JL Display Track mA Amperage ************** //