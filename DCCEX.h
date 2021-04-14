/*
 *  (c) 2020 Chris Harlow. All rights reserved.
 *  (c) 2021 Fred Decker.  All rights reserved.
 *  (c) 2020 Harald Barth. All rights reserved.
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

// This include is intended to visually simplify the .ino for the end users.
// If there were any #ifdefs required they are much better handled in here.  

#ifndef DCCEX_h
#define DCCEX_h

#include "defines.h"
#include "DCC.h"
#include "DIAG.h"
#include "DCCEXParser.h"
#include "version.h"
#include "WifiInterface.h"
#if ETHERNET_ON == true
#include "EthernetInterface.h"
#endif
#include "LCD_Implementation.h"
#include "LCN.h"
#include "freeMemory.h"

#if __has_include ( "myAutomation.h")
  #include "RMFT.h"
  #define RMFT_ACTIVE
#endif
    
#endif
