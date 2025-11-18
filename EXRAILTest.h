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
  WAIT_WHILE_RED(1) // user do </GREEN 1> to continue
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
  DONE

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
