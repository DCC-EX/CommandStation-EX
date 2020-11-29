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
#include "freeMemory.h"
#include <Arduino.h>

#endif
