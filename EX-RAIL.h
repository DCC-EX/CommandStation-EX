#ifndef RMFT_H
#define RMFT_H

#if defined(RMFT_ACTIVE)
 #include "RMFT2.h"

  class RMFT {
    public:
      static void inline begin() {RMFT2::begin();}
      static void inline loop() {RMFT2::loop();}
  };

  #include "RMFTMacros.h"
  
#else 
  // Dummy RMFT 
  class RMFT {
    public:
      static void inline begin() {}
      static void inline loop() {}
  };
#endif
#endif
