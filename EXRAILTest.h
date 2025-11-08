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
  IFROUTE_ENABLED(7000) PRINT("700df failed") ENDIF

  ROUTE_HIDDEN(7000)
  IFROUTE_HIDDEN(7000) ELSE PRINT("7001g failed") ENDIF
  
  ROUTE_ENABLED(7000)
  