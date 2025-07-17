// ************* OLED JL Display Track mA Amperage  **************** //
// by Herb Morton    [Ash]                          March 23, 2025 
//    Colin Murdoch  [ColinM] 

//  Inside your config.h file First edit OLED max char rows set to 17
// #define MAX_CHARACTER_ROWS 17

// myAutomation.h
// Reporting power status and mA for each track on the LCD
HAL(Bitmap,8236,1) // create flag 8236
AUTOSTART DELAY(5000) 
 ROUTE("TRACKSTATUS"_hk, "Resume/Pause JL Display")
  IF(8236) 
    RESET(8236)
     ROUTE_CAPTION("TRACKSTATUS"_hk, "Paused") ROUTE_INACTIVE("TRACKSTATUS"_hk)
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
  SET(8236) 
   ROUTE_CAPTION("TRACKSTATUS"_hk, "Running") ROUTE_ACTIVE("TRACKSTATUS"_hk)
    PRINT("Resume JL Display")
   FOLLOW("PAUSETRACKSTATUS"_hk)
  SEQUENCE("PAUSETRACKSTATUS"_hk)
   PARSE("<JL 0 8>")  // screen 0  start on line 8
    PRINT("\n")
    DELAY(3000)
  IF(8236) FOLLOW("PAUSETRACKSTATUS"_hk) ENDIF
  DONE
// ************ End OLED JL Display Track mA Amperage ************** //