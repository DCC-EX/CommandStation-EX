/*
 *  © 2020-2025 Chris Harlow. All rights reserved.
 *  © 2022-2023 Colin Murdoch
 *  © 2023 Harald Barth
 *  © 2025 Morten Nielsen
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
 *  along with CommandStation.  If not, see https://www.gnu.org/licenses/.
 */

// This file cleans and resets the EXRAIL Macros.
// It is used between passes to reduce complexity in EXRAILMacros.h
// DO NOT add an include guard to this file.

// Undefine all RMFT macros
#undef ACTIVATE
#undef ACTIVATEL
#undef AFTER
#undef AFTEROVERLOAD
#undef ALIAS
#undef AMBER
#undef ANOUT
#undef ASPECT
#undef AT
#undef ATGTE
#undef ATLT
#undef ATTIMEOUT
#undef AUTOMATION 
#undef AUTOSTART
#undef BLINK
#undef BUILD_CONSIST
#undef BREAK_CONSIST
#undef BROADCAST
#undef CALL 
#undef CLEAR_STASH
#undef CLEAR_ALL_STASH
#undef CLEAR_ANY_STASH
#undef CLOSE 
#undef CONFIGURE_SERVO
#undef DCC_SIGNAL
#undef DCCX_SIGNAL
#undef DCC_TURNTABLE
#undef DEACTIVATE
#undef DEACTIVATEL
#undef DELAY
#undef DELAYMINS
#undef DELAYRANDOM 
#undef DONE
#undef DRIVE
#undef ELSE
#undef ENDEXRAIL 
#undef ENDIF  
#undef ENDTASK
#undef ESTOP
#undef ESTOPALL
#undef ESTOP_PAUSE
#undef ESTOP_RESUME
#undef EXRAIL
#undef EXTT_TURNTABLE
#undef FADE
#undef FOFF
#undef FOLLOW 
#undef FON 
#undef FORGET
#undef FTOGGLE
#undef FREE
#undef FREEALL
#undef FWD 
#undef GREEN
#undef HAL
#undef HAL_IGNORE_DEFAULTS
#undef IF 
#undef IFAMBER
#undef IFCLOSED
#undef IFGREEN
#undef IFGTE
#undef IFLOCO
#undef IFLT
#undef IFNOT
#undef IFRANDOM 
#undef IFRED
#undef IFRESERVE
#undef IFSTASH
#undef IFSTASHED_HERE
#undef IFTHROWN
#undef IFTIMEOUT
#undef IFTTPOSITION
#undef IFRE
#undef IFROUTE_ACTIVE
#undef IFROUTE_INACTIVE
#undef IFROUTE_HIDDEN
#undef IFROUTE_DISABLED
#undef IFBITMAP_ALL
#undef IFBITMAP_ANY
#undef INVERT_DIRECTION 
#undef JMRI_SENSOR
#undef JMRI_SENSOR_NOPULLUP
#undef JOIN 
#undef KILLALL
#undef LATCH 
#undef LCD 
#undef SCREEN
#undef LCC 
#undef LCCX 
#undef LCN 
#undef MOMENTUM
#undef MOVETT
#undef NEOPIXEL
#undef NEOPIXEL_OFF
#undef NEOPIXEL_SIGNAL
#undef ACON
#undef ACOF
#undef ONACON
#undef ONACOF
#undef MESSAGE
#undef ONACTIVATE
#undef ONACTIVATEL
#undef ONAMBER
#undef ONBLOCKENTER
#undef ONBLOCKEXIT
#undef ONDEACTIVATE
#undef ONDEACTIVATEL 
#undef ONCLOSE
#undef ONLCC
#undef ONTIME
#undef ONCLOCKTIME
#undef ONCLOCKMINS
#undef ONOVERLOAD
#undef ONRAILSYNCON
#undef ONRAILSYNCOFF
#undef ONGREEN
#undef ONRED
#undef ONROTATE
#undef ONBUTTON
#undef ONSENSOR
#undef ONTHROW 
#undef ONBITMAP
#undef ONCHANGE
#undef PARSE
#undef PAUSE
#undef PICKUP_STASH
#undef PIN_TURNOUT
#undef PLAY_EQ
#undef PLAY_FOLDER
#undef PLAY_PAUSE
#undef PLAY_REPEAT
#undef PLAY_RESET
#undef PLAY_RESUME
#undef PLAY_STOP
#undef PLAY_TRACK
#undef PLAY_VOLUME
#undef PRINT
#undef POM
#undef POWEROFF
#undef POWERON
#undef RANDOM_CALL
#undef RANDOM_FOLLOW
#undef READ_LOCO 
#undef RED 
#undef RESERVE
#undef RESERVE_NOESTOP
#undef RESET 
#undef RESTORE_SPEED
#undef RESUME 
#undef RETURN 
#undef REV
#undef ROSTER
#undef ROTATE
#undef ROTATE_DCC
#undef ROUTE
#undef ROUTE_ACTIVE
#undef ROUTE_INACTIVE
#undef ROUTE_HIDDEN
#undef ROUTE_DISABLED
#undef ROUTE_CAPTION
#undef SAVE_SPEED
#undef SENDLOCO 
#undef SEQUENCE 
#undef SERIAL 
#undef SERIAL1 
#undef SERIAL2 
#undef SERIAL3 
#undef SERIAL4 
#undef SERIAL5 
#undef SERIAL6 
#undef SERVO 
#undef SERVO2 
#undef SERVO_TURNOUT 
#undef SERVO_SIGNAL
#undef SET
#undef SET_TRACK
#undef SET_POWER
#undef SETLOCO 
#undef SETFREQ
#undef SIGNAL 
#undef SIGNALH 
#undef SPEED 
#undef START
#undef START_SHARED
#undef START_SEND
#undef STASH
#undef STEALTH
#undef STEALTH_GLOBAL
#undef STOP 
#undef THROW
#undef TOGGLE_TURNOUT
#undef TT_ADDPOSITION
#undef TURNOUT 
#undef TURNOUTL
#undef UNJOIN
#undef UNLATCH 
#undef VIRTUAL_SIGNAL
#undef VIRTUAL_TURNOUT
#undef WAITFOR
#undef WAIT_WHILE_RED
#ifndef IO_NO_HAL
#undef BITMAP_AND
#undef BITMAP_OR
#undef BITMAP_XOR
#undef BITMAP_INC
#undef BITMAP_DEC
#undef WAITFORTT
#endif
#undef WITHROTTLE
#undef XFOFF
#undef XFON
#undef XFTOGGLE
#undef XPOM
#undef XREV
#undef XFWD
#undef XSAVE_SPEED
#undef XRESTORE_SPEED
#undef ZTEST
#undef ZTEST2
#undef ZTEST3

#ifndef RMFT2_UNDEF_ONLY
#include "EXRAIL2MacroBase.h"
#endif
