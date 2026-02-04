// This file contains various EXRAIL tests.
// It will need to be #included in a myAutomation.h to be executed.

ROUTE(7000, "Route state tests")

  // Test route state management
  PRINT("Test 7001")
  ROUTE_ACTIVE(7000)
  IFROUTE_ACTIVE(7000) ELSE PRINT("7001a failed") ENDIF
  IFROUTE_INACTIVE(7000) PRINT("7001b failed") ENDIF
  
  ROUTE_INACTIVE(7000)
  IFROUTE_INACTIVE(7000) ELSE PRINT("7001c failed") ENDIF
  IFROUTE_ACTIVE(7000) PRINT("700d failed") ENDIF

  ROUTE_DISABLED(7000)
  IFROUTE_DISABLED(7000) ELSE PRINT("7001e failed") ENDIF
  IFROUTE_ACTIVE(7000) PRINT("700df failed") ENDIF
  IFROUTE_INACTIVE(7000) PRINT("700dg failed") ENDIF

  ROUTE_HIDDEN(7000)
  IFROUTE_HIDDEN(7000) ELSE PRINT("7001h failed") ENDIF
  
  ROUTE_INACTIVE(7000)
  DONE


  // Test WAIT_WHILE_RED
  VIRTUAL_SIGNAL(1)
  ROUTE(7100, "7100 WAIT_WHILE_RED test")
  SETLOCO(3)
  SPEED(10)
  RED(1)
  PRINT("7100 Waiting at signal 1")
  PRINT("7100 Tester please set signal 1 GREEN or AMBER to continue")
  WAIT_WHILE_RED(1)
  SPEED(10)
  PRINT("7100 Resumed at signal 1")
  DONE

  // TEST new FREEALL command
  ROUTE(7200,"7200 Test FREEALL")
  RESERVE(1)
  RESERVE(255)
  PARSE("</>")
  FREEALL
  PARSE("</>")

  IFRESERVE(1)
    PRINT("7200 FREEALL test worked")
    ELSE 
    PRINT("7200 FREEALL test fail")
    ENDIF
  DONE

  // Test SEND_LOCO_X and SEND_LOCO_S
  ROUTE(7300,"7300 Test SEND_LOCO_ variants")
  SETLOCO(5) 
  
  // Share loco and check we doidnt lose it
  START_SHARED(7301)
  IFLOCO(5) ELSE PRINT("7300 START_SHARED failed") ENDIF

  // transfer loco and check we do lose it
  START_SEND(7302)
  IFLOCO(0) ELSE PRINT("7300 START_SEND failed") ENDIF
  DONE
  
  SEQUENCE(7301)
    IFLOCO(5)
      PRINT("7301 START_SHARED received")
      ELSE 
      PRINT("7301 START_SHARED failed")
      ENDIF
    DONE   
    SEQUENCE(7302)
    IFLOCO(5)
      PRINT("7302 START_SEND received")
      ELSE 
      PRINT("7302 START_SEND failed")
      ENDIF
    DONE  

    SEQUENCE(7400)
      SETLOCO(3)
      SPEED(20)
      SAVE_SPEED
      SPEED(40)
      PRINT("7401 Speed changed to 40")
      RESTORE_SPEED
      PRINT("7401 Speed restored")
      XFWD(4,20)
      XSAVE_SPEED(4)
      XFWD(4,40)
      PRINT("7401 Speed 4 changed to 40")
      XRESTORE_SPEED(4)
      PRINT("7401 Speed 4 restored")
      DONE
      
SEQUENCE(7500)
  PRINT("7500 Testing RANDOM_CALL and RANDOM_FOLLOW")
  RANDOM_CALL(7501, 7502, 7503)
  PRINT("7500 Returned from RANDOM_CALL")
  RANDOM_FOLLOW(7511, 7512, 7513)
  PRINT("7500 ERROR Returned from RANDOM_FOLLOW")
  DONE

  SEQUENCE(7501) PRINT("7501 RANDOM_CALL") RETURN
  SEQUENCE(7502) PRINT("7502 RANDOM_CALL") RETURN
  SEQUENCE(7503) PRINT("7503 RANDOM_CALL") RETURN

  SEQUENCE(7511) PRINT("7511 RANDOM_FOLLOW") DONE
  SEQUENCE(7512) PRINT("7512 RANDOM_FOLLOW") DONE
  SEQUENCE(7513) PRINT("7513 RANDOM_FOLLOW") DONE
  
  ROUTE(7600, "7600 Test CONSIST")
    PARSE("7600 Create 6 77 88 -99")
    SETLOCO(6)
    BUILD_CONSIST(77)
    BUILD_CONSIST(88)
    BUILD_CONSIST(-99)
    PARSE("<^>")  // display consists 
    PRINT("break up") 
    BREAK_CONSIST
    PARSE("<^>")  // display consists  
  DONE

  HAL(Bitmap,870,10)
  ROUTE(7700,"Set 870 and wait")
    SET(870)
    AT(-870)
    PRINT("870 reset")
  DONE
  ROUTE(7701,"ResetS 870 and wait")
    RESET(870)
    AT(870)
    PRINT("870 set")
  DONE

  ROUTE(7800, "ZTESTS")
  PRINT("ZTESTS starting")
  ZTEST("<t 3 5 1>",DCC::getLocoSpeedByte(3)==(128+6))
  ZTEST("<t 3 5 0>",DCC::getLocoSpeedByte(3)==(6))
  ZTEST("<-3>",DCC::getLocoSpeedByte(3)==(128))  
  ZTEST2("<$>","<X>\n")
  DONE

ROUTE(7900,"7900 Test IFSTASHED_HERE")
   SETLOCO(4)  // set loco 4
   STASH(200)  // stash loco 4 in stash 2
    
// loco is 4
  IFSTASHED_HERE(100) // should be false
      PRINT("FAIL Loco 4 in stash 100")
  ELSE
      PRINT("OK: loco 4 is not in stash 100")   
  ENDIF

SETLOCO(3)
IFSTASHED_HERE(200) // should be false
      PRINT("FAIL Loco 3 in stash 200")
ELSE
      PRINT("OK: loco 3 is not in stash 200")   
ENDIF

IFSTASHED_HERE(100) // should be true
      PRINT("OK: Loco 3 is in stash 100")
ELSE
      PRINT("FAIL: loco 3 not in stash 100")
ENDIF
DONE

ROUTE(8000,"8000 ESTOP_PAUSE test")
   SETLOCO(3)  // set loco 3
   FWD(20)    //
   ESTOP_PAUSE
   SETLOCO(0)  // prevent DONE stopping it
   DONE
ROUTE(8001,"8001 ESTOP_RESUME test")
   ESTOP_RESUME
   ZTEST("<D CABS>",DCC::getLocoSpeedByte(3)==(128+20))
   DONE

   