#ifndef EXRAIL_H
#define EXRAIL_H

#if defined(EXRAIL_ACTIVE)
 #include "EXRAIL2.h"

  class RMFT {
    public:
      static void inline begin() {RMFT2::begin();}
      static void inline loop() {RMFT2::loop();}
  };

  #include "EXRAILMacros.h"
  
#else 
  // Dummy RMFT 
  class RMFT {
    public:
      static void inline begin() {}
      static void inline loop() {}
  };
#endif
#endif
