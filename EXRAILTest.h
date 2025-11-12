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


  VIRTUAL_SIGNAL(1)

  // Test WAIT_WHILE_RED
  ROUTE(7100, "WAIT_WHILE_RED test")
  SETLOCO(3)
  SPEED(10)
  RED(1)
  PRINT("Waiting at signal 1")
  WAIT_WHILE_RED(1) // user do </GREEN 1> to continue
  SPEED(10)
  PRINT("Resumed at signal 1")
  DONE

  ROUTE(7200,"Test FREEALL")
  RESERVE(1)
  RESERVE(255)
  PARSE("</>")
  FREEALL
  PARSE("</>")
  DONE

