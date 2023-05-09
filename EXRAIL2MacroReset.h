/*
 *  © 2020-2022 Chris Harlow. All rights reserved.
 *  © 2022 Colin Murdoch
 *  © 2023 Harald Barth
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

// This file cleans and resets the RMFT2 Macros.
// It is used between passes to reduce complexity in RMFT2Macros.h
// DO NOT add an include guard to this file.

// Undefine all RMFT macros
#undef ACTIVATE
#undef ACTIVATEL
#undef AFTER
#undef ALIAS
#undef AMBER
#undef ANOUT
#undef AT
#undef ATGTE
#undef ATLT
#undef ATTIMEOUT
#undef AUTOMATION 
#undef AUTOSTART
#undef BROADCAST
#undef CALL 
#undef CLOSE 
#undef DCC_SIGNAL
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
#undef EXRAIL  
#undef FADE
#undef FOFF
#undef FOLLOW 
#undef FON 
#undef FORGET
#undef FREE 
#undef FWD 
#undef GREEN
#undef HAL
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
#undef IFTHROWN
#undef IFTIMEOUT
#undef IFRE
#undef INVERT_DIRECTION 
#undef JOIN 
#undef KILLALL
#undef LATCH 
#undef LCD 
#undef SCREEN
#undef LCN 
#undef MOVETT
#undef ONACTIVATE
#undef ONACTIVATEL
#undef ONAMBER
#undef ONDEACTIVATE
#undef ONDEACTIVATEL 
#undef ONCLOSE
#undef ONTIME
#undef ONCLOCKTIME
#undef ONCLOCKMINS
#undef ONGREEN
#undef ONRED
#undef ONTHROW 
#undef ONCHANGE
#undef PARSE
#undef PAUSE
#undef PIN_TURNOUT 
#undef PRINT
#ifndef DISABLE_PROG
#undef POM
#endif
#undef POWEROFF
#undef POWERON
#undef READ_LOCO 
#undef RED 
#undef RESERVE 
#undef RESET 
#undef RESUME 
#undef RETURN 
#undef REV
#undef ROSTER 
#undef ROUTE
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
#undef SETLOCO 
#undef SIGNAL 
#undef SIGNALH 
#undef SPEED 
#undef START 
#undef STOP 
#undef THROW  
#undef TURNOUT 
#undef TURNOUTL
#undef UNJOIN
#undef UNLATCH 
#undef VIRTUAL_SIGNAL
#undef VIRTUAL_TURNOUT
#undef WAITFOR
#undef WITHROTTLE
#undef XFOFF
#undef XFON

#ifndef RMFT2_UNDEF_ONLY
#define ACTIVATE(addr,subaddr)
#define ACTIVATEL(addr)
#define AFTER(sensor_id)
#define ALIAS(name,value...)
#define AMBER(signal_id)
#define ANOUT(vpin,value,param1,param2)
#define AT(sensor_id)
#define ATGTE(sensor_id,value) 
#define ATLT(sensor_id,value) 
#define ATTIMEOUT(sensor_id,timeout_ms)
#define AUTOMATION(id,description) 
#define AUTOSTART
#define BROADCAST(msg)
#define CALL(route) 
#define CLOSE(id) 
#define DCC_SIGNAL(id,add,subaddr)
#define DEACTIVATE(addr,subaddr)
#define DEACTIVATEL(addr)
#define DELAY(mindelay)
#define DELAYMINS(mindelay)
#define DELAYRANDOM(mindelay,maxdelay) 
#define DONE
#define DRIVE(analogpin)
#define ELSE
#define ENDEXRAIL 
#define ENDIF  
#define ENDTASK
#define ESTOP 
#define EXRAIL  
#define FADE(pin,value,ms)
#define FOFF(func)
#define FOLLOW(route) 
#define FON(func)
#define FORGET
#define FREE(blockid) 
#define FWD(speed) 
#define GREEN(signal_id)
#define HAL(haltype,params...)
#define IF(sensor_id) 
#define IFAMBER(signal_id)
#define IFCLOSED(turnout_id) 
#define IFGREEN(signal_id)
#define IFGTE(sensor_id,value) 
#define IFLOCO(loco_id)
#define IFLT(sensor_id,value) 
#define IFNOT(sensor_id)
#define IFRANDOM(percent)
#define IFRED(signal_id)
#define IFTHROWN(turnout_id) 
#define IFRESERVE(block)
#define IFTIMEOUT
#define IFRE(sensor_id,value)
#define INVERT_DIRECTION 
#define JOIN 
#define KILLALL
#define LATCH(sensor_id) 
#define LCD(row,msg)
#define SCREEN(display,row,msg)
#define LCN(msg) 
#define MOVETT(id,steps,activity)
#define ONACTIVATE(addr,subaddr)
#define ONACTIVATEL(linear)
#define ONAMBER(signal_id) 
#define ONTIME(value)
#define ONCLOCKTIME(hours,mins)
#define ONCLOCKMINS(mins)
#define ONDEACTIVATE(addr,subaddr)
#define ONDEACTIVATEL(linear) 
#define ONCLOSE(turnout_id)
#define ONGREEN(signal_id) 
#define ONRED(signal_id) 
#define ONTHROW(turnout_id) 
#define ONCHANGE(sensor_id)
#define PAUSE
#define PIN_TURNOUT(id,pin,description...) 
#define PRINT(msg) 
#define PARSE(msg)
#ifndef DISABLE_PROG
#define POM(cv,value)
#endif
#define POWEROFF
#define POWERON
#define READ_LOCO 
#define RED(signal_id) 
#define RESERVE(blockid) 
#define RESET(pin) 
#define RESUME 
#define RETURN 
#define REV(speed) 
#define ROUTE(id,description)
#define ROSTER(cab,name,funcmap...)
#define SENDLOCO(cab,route) 
#define SEQUENCE(id) 
#define SERIAL(msg) 
#define SERIAL1(msg) 
#define SERIAL2(msg) 
#define SERIAL3(msg) 
#define SERIAL4(msg) 
#define SERIAL5(msg) 
#define SERIAL6(msg) 
#define SERVO(id,position,profile) 
#define SERVO2(id,position,duration) 
#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) 
#define SET(pin) 
#define SET_TRACK(track,mode)
#define SETLOCO(loco) 
#define SIGNAL(redpin,amberpin,greenpin) 
#define SIGNALH(redpin,amberpin,greenpin) 
#define SPEED(speed) 
#define START(route) 
#define STOP 
#define THROW(id)  
#define TURNOUT(id,addr,subaddr,description...) 
#define TURNOUTL(id,addr,description...) 
#define UNJOIN 
#define UNLATCH(sensor_id) 
#define VIRTUAL_SIGNAL(id) 
#define VIRTUAL_TURNOUT(id,description...) 
#define WAITFOR(pin)
#define WITHROTTLE(msg)
#define XFOFF(cab,func)
#define XFON(cab,func)
#endif
