/*
 *  © 2021 Fred Decker
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

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
