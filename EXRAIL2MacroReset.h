/*
 *  © 2020-2022 Chris Harlow. All rights reserved.
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

// This file cleans and resets the RMFT2 Macros.
// It is used between passes to reduce complexity in RMFT2Macros.h
// DO NOT add an include guard to this file.

// Doxygen comments in this file are intended for the EXRAIL end user.

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
#undef BROADCAST
#undef CALL 
#undef CLEAR_STASH
#undef CLEAR_ALL_STASH
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
#undef EXRAIL
#undef EXTT_TURNTABLE
#undef FADE
#undef FOFF
#undef FOLLOW 
#undef FON 
#undef FORGET
#undef FTOGGLE
#undef FREE 
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
#undef IFTHROWN
#undef IFTIMEOUT
#undef IFTTPOSITION
#undef IFRE
#undef INVERT_DIRECTION 
#undef JMRI_SENSOR
#undef JOIN 
#undef KILLALL
#undef LATCH 
#undef LCD 
#undef SCREEN
#undef LCC 
#undef LCCX 
#undef LCN 
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
#undef ONDEACTIVATE
#undef ONDEACTIVATEL 
#undef ONCLOSE
#undef ONLCC
#undef ONTIME
#undef ONCLOCKTIME
#undef ONCLOCKMINS
#undef ONOVERLOAD
#undef ONGREEN
#undef ONRED
#undef ONROTATE
#undef ONBUTTON
#undef ONSENSOR
#undef ONTHROW 
#undef ONCHANGE
#undef PARSE
#undef PAUSE
#undef PICKUP_STASH
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
#undef ROTATE
#undef ROTATE_DCC
#undef ROUTE
#undef ROUTE_ACTIVE
#undef ROUTE_INACTIVE
#undef ROUTE_HIDDEN
#undef ROUTE_DISABLED
#undef ROUTE_CAPTION
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
#ifndef IO_NO_HAL
#undef WAITFORTT
#endif
#undef WITHROTTLE
#undef XFOFF
#undef XFON
#undef XFTOGGLE
#undef XREV
#undef XFWD

#ifndef RMFT2_UNDEF_ONLY
/**
 * @def ACTIVATE(addr,subaddr)
 * @brief 
 * @param addr 
 * @param subaddr 
 */
#define ACTIVATE(addr,subaddr)
/**
 * @def ACTIVATEL(addr)
 * @brief 
 * @param addr 
 */
#define ACTIVATEL(addr)
/**
 * @def AFTER(sensor_id,timer...)
 * @brief 
 * @param sensor_id 
 * @param timer... 
 */
#define AFTER(sensor_id,timer...)
/**
 * @def AFTEROVERLOAD(track_id)
 * @brief 
 * @param track_id 
 */
#define AFTEROVERLOAD(track_id)
/**
 * @def ALIAS(name,value...)
 * @brief 
 * @param name 
 * @param value... 
 */
#define ALIAS(name,value...)
/**
 * @def AMBER(signal_id)
 * @brief 
 * @param signal_id 
 */
#define AMBER(signal_id)
/**
 * @def ANOUT(vpin,value,param1,param2)
 * @brief 
 * @param vpin 
 * @param value 
 * @param param1 
 * @param param2 
 */
#define ANOUT(vpin,value,param1,param2)
/**
 * @def AT(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define AT(sensor_id)
/**
 * @def ASPECT(address,value)
 * @brief 
 * @param address 
 * @param value 
 */
#define ASPECT(address,value)
/**
 * @def ATGTE(sensor_id,value)
 * @brief 
 * @param sensor_id 
 * @param value 
 */
#define ATGTE(sensor_id,value) 
/**
 * @def ATLT(sensor_id,value)
 * @brief 
 * @param sensor_id 
 * @param value 
 */
#define ATLT(sensor_id,value) 
/**
 * @def ATTIMEOUT(sensor_id,timeout_ms)
 * @brief 
 * @param sensor_id 
 * @param timeout_ms 
 */
#define ATTIMEOUT(sensor_id,timeout_ms)
/**
 * @def AUTOMATION(id,description)
 * @brief 
 * @param id 
 * @param description 
 */
#define AUTOMATION(id,description) 
/**
 * @def AUTOSTART
 * @brief 
 */
#define AUTOSTART
/**
 * @def BLINK(vpin,onDuty,offDuty)
 * @brief 
 * @param vpin 
 * @param onDuty 
 * @param offDuty 
 */
#define BLINK(vpin,onDuty,offDuty)
/**
 * @def BROADCAST(msg)
 * @brief 
 * @param msg 
 */
#define BROADCAST(msg)
/**
 * @def CALL(route)
 * @brief 
 * @param route 
 */
#define CALL(route)
/**
 * @def CLEAR_STASH(id)
 * @brief 
 * @param id 
 */
#define CLEAR_STASH(id)
/**
 * @def CLEAR_ALL_STASH(id)
 * @brief 
 * @param id 
 */
#define CLEAR_ALL_STASH(id)
/**
 * @def CLOSE(id)
 * @brief 
 * @param id 
 */
#define CLOSE(id)
/**
 * @def CONFIGURE_SERVO(vpin,pos1,pos2,profile)
 * @brief 
 * @param vpin 
 * @param pos1 
 * @param pos2 
 * @param profile 
 */
#define CONFIGURE_SERVO(vpin,pos1,pos2,profile) 
/**
 * @def DCC_SIGNAL(id,add,subaddr)
 * @brief 
 * @param id 
 * @param add 
 * @param subaddr 
 */
#define DCC_SIGNAL(id,add,subaddr)
/**
 * @def DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect)
 * @brief 
 * @param id 
 * @param redAspect 
 * @param amberAspect 
 * @param greenAspect 
 */
#define DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect)
/**
 * @def DCC_TURNTABLE(id,home,description...)
 * @brief 
 * @param id 
 * @param home 
 * @param description... 
 */
#define DCC_TURNTABLE(id,home,description...)
/**
 * @def DEACTIVATE(addr,subaddr)
 * @brief 
 * @param addr 
 * @param subaddr 
 */
#define DEACTIVATE(addr,subaddr)
/**
 * @def DEACTIVATEL(addr)
 * @brief 
 * @param addr 
 */
#define DEACTIVATEL(addr)
/**
 * @def DELAY(mindelay)
 * @brief 
 * @param mindelay 
 */
#define DELAY(mindelay)
/**
 * @def DELAYMINS(mindelay)
 * @brief 
 * @param mindelay 
 */
#define DELAYMINS(mindelay)
/**
 * @def DELAYRANDOM(mindelay,maxdelay)
 * @brief 
 * @param mindelay 
 * @param maxdelay 
 */
#define DELAYRANDOM(mindelay,maxdelay) 
/**
 * @def DONE
 * @brief 
 */
#define DONE
/**
 * @def DRIVE(analogpin)
 * @brief 
 * @param analogpin 
 */
#define DRIVE(analogpin)
/**
 * @def ELSE
 * @brief 
 */
#define ELSE
/**
 * @def ENDEXRAIL
 * @brief 
 */
#define ENDEXRAIL 
/**
 * @def ENDIF
 * @brief 
 */
#define ENDIF  
/**
 * @def ENDTASK
 * @brief 
 */
#define ENDTASK
/**
 * @def ESTOP
 * @brief 
 */
#define ESTOP 
/**
 * @def EXRAIL
 * @brief 
 */
#define EXRAIL
/**
 * @def EXTT_TURNTABLE(id,vpin,home,description...)
 * @brief 
 * @param id 
 * @param vpin 
 * @param home 
 * @param description... 
 */
#define EXTT_TURNTABLE(id,vpin,home,description...)
/**
 * @def FADE(pin,value,ms)
 * @brief 
 * @param pin 
 * @param value 
 * @param ms 
 */
#define FADE(pin,value,ms)
/**
 * @def FOFF(func)
 * @brief 
 * @param func 
 */
#define FOFF(func)
/**
 * @def FOLLOW(route)
 * @brief 
 * @param route 
 */
#define FOLLOW(route) 
/**
 * @def FON(func)
 * @brief 
 * @param func 
 */
#define FON(func)
/**
 * @def FORGET
 * @brief 
 */
#define FORGET
/**
 * @def FREE(blockid)
 * @brief 
 * @param blockid 
 */
#define FREE(blockid) 
/**
 * @def FTOGGLE(func)
 * @brief 
 * @param func 
 */
#define FTOGGLE(func)
/**
 * @def FWD(speed)
 * @brief 
 * @param speed 
 */
#define FWD(speed) 
/**
 * @def GREEN(signal_id)
 * @brief 
 * @param signal_id 
 */
#define GREEN(signal_id)
/**
 * @def HAL(haltype,params...)
 * @brief 
 * @param haltype 
 * @param params... 
 */
#define HAL(haltype,params...)
/**
 * @def HAL_IGNORE_DEFAULTS
 * @brief 
 */
#define HAL_IGNORE_DEFAULTS
/**
 * @def IF(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define IF(sensor_id) 
/**
 * @def IFAMBER(signal_id)
 * @brief 
 * @param signal_id 
 */
#define IFAMBER(signal_id)
/**
 * @def IFCLOSED(turnout_id)
 * @brief 
 * @param turnout_id 
 */
#define IFCLOSED(turnout_id) 
/**
 * @def IFGREEN(signal_id)
 * @brief 
 * @param signal_id 
 */
#define IFGREEN(signal_id)
/**
 * @def IFGTE(sensor_id,value)
 * @brief 
 * @param sensor_id 
 * @param value 
 */
#define IFGTE(sensor_id,value) 
/**
 * @def IFLOCO(loco_id)
 * @brief 
 * @param loco_id 
 */
#define IFLOCO(loco_id)
/**
 * @def IFLT(sensor_id,value)
 * @brief 
 * @param sensor_id 
 * @param value 
 */
#define IFLT(sensor_id,value) 
/**
 * @def IFNOT(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define IFNOT(sensor_id)
/**
 * @def IFRANDOM(percent)
 * @brief 
 * @param percent 
 */
#define IFRANDOM(percent)
/**
 * @def IFRED(signal_id)
 * @brief 
 * @param signal_id 
 */
#define IFRED(signal_id)
/**
 * @def IFTHROWN(turnout_id)
 * @brief 
 * @param turnout_id 
 */
#define IFTHROWN(turnout_id) 
/**
 * @def IFRESERVE(block)
 * @brief 
 * @param block 
 */
#define IFRESERVE(block)
/**
 * @def IFTIMEOUT
 * @brief 
 */
#define IFTIMEOUT
/**
 * @def IFTTPOSITION(turntable_id,position)
 * @brief 
 * @param turntable_id 
 * @param position 
 */
#define IFTTPOSITION(turntable_id,position)
/**
 * @def IFRE(sensor_id,value)
 * @brief 
 * @param sensor_id 
 * @param value 
 */
#define IFRE(sensor_id,value)
/**
 * @def INVERT_DIRECTION
 * @brief 
 */
#define INVERT_DIRECTION 
/**
 * @def JMRI_SENSOR(vpin,count...)
 * @brief 
 * @param vpin 
 * @param count... 
 */
#define JMRI_SENSOR(vpin,count...)
/**
 * @def JOIN
 * @brief 
 */
#define JOIN 
/**
 * @def KILLALL
 * @brief 
 */
#define KILLALL
/**
 * @def LATCH(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define LATCH(sensor_id)
/**
 * @def LCC(eventid)
 * @brief 
 * @param eventid 
 */
#define LCC(eventid) 
/**
 * @def LCCX(senderid,eventid)
 * @brief 
 * @param senderid 
 * @param eventid 
 */
#define LCCX(senderid,eventid) 
/**
 * @def LCD(row,msg)
 * @brief 
 * @param row 
 * @param msg 
 */
#define LCD(row,msg)
/**
 * @def SCREEN(display,row,msg)
 * @brief 
 * @param display 
 * @param row 
 * @param msg 
 */
#define SCREEN(display,row,msg)
/**
 * @def LCN(msg)
 * @brief 
 * @param msg 
 */
#define LCN(msg) 
/**
 * @def MESSAGE(msg)
 * @brief 
 * @param msg 
 */
#define MESSAGE(msg)
/**
 * @def MOVETT(id,steps,activity)
 * @brief 
 * @param id 
 * @param steps 
 * @param activity 
 */
#define MOVETT(id,steps,activity)
/**
 * @def NEOPIXEL(id,r,g,b,count...)
 * @brief 
 * @param id 
 * @param r 
 * @param g 
 * @param b 
 * @param count... 
 */
#define NEOPIXEL(id,r,g,b,count...)
/**
 * @def NEOPIXEL_SIGNAL(sigid,redcolour,ambercolour,greencolour)
 * @brief 
 * @param sigid 
 * @param redcolour 
 * @param ambercolour 
 * @param greencolour 
 */
#define NEOPIXEL_SIGNAL(sigid,redcolour,ambercolour,greencolour)
/**
 * @def ACON(eventid)
 * @brief 
 * @param eventid 
 */
#define ACON(eventid)
/**
 * @def ACOF(eventid)
 * @brief 
 * @param eventid 
 */
#define ACOF(eventid)
/**
 * @def ONACON(eventid)
 * @brief 
 * @param eventid 
 */
#define ONACON(eventid)
/**
 * @def ONACOF(eventid)
 * @brief 
 * @param eventid 
 */
#define ONACOF(eventid)
/**
 * @def ONACTIVATE(addr,subaddr)
 * @brief 
 * @param addr 
 * @param subaddr 
 */
#define ONACTIVATE(addr,subaddr)
/**
 * @def ONACTIVATEL(linear)
 * @brief 
 * @param linear 
 */
#define ONACTIVATEL(linear)
/**
 * @def ONAMBER(signal_id)
 * @brief 
 * @param signal_id 
 */
#define ONAMBER(signal_id) 
/**
 * @def ONTIME(value)
 * @brief 
 * @param value 
 */
#define ONTIME(value)
/**
 * @def ONCLOCKTIME(hours,mins)
 * @brief 
 * @param hours 
 * @param mins 
 */
#define ONCLOCKTIME(hours,mins)
/**
 * @def ONCLOCKMINS(mins)
 * @brief 
 * @param mins 
 */
#define ONCLOCKMINS(mins)
/**
 * @def ONOVERLOAD(track_id)
 * @brief 
 * @param track_id 
 */
#define ONOVERLOAD(track_id)
/**
 * @def ONDEACTIVATE(addr,subaddr)
 * @brief 
 * @param addr 
 * @param subaddr 
 */
#define ONDEACTIVATE(addr,subaddr)
/**
 * @def ONDEACTIVATEL(linear)
 * @brief 
 * @param linear 
 */
#define ONDEACTIVATEL(linear) 
/**
 * @def ONCLOSE(turnout_id)
 * @brief 
 * @param turnout_id 
 */
#define ONCLOSE(turnout_id)
/**
 * @def ONLCC(sender,event)
 * @brief 
 * @param sender 
 * @param event 
 */
#define ONLCC(sender,event)
/**
 * @def ONGREEN(signal_id)
 * @brief 
 * @param signal_id 
 */
#define ONGREEN(signal_id) 
/**
 * @def ONRED(signal_id)
 * @brief 
 * @param signal_id 
 */
#define ONRED(signal_id)
/**
 * @def ONROTATE(turntable_id)
 * @brief 
 * @param turntable_id 
 */
#define ONROTATE(turntable_id)
/**
 * @def ONTHROW(turnout_id)
 * @brief 
 * @param turnout_id 
 */
#define ONTHROW(turnout_id) 
/**
 * @def ONCHANGE(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define ONCHANGE(sensor_id)
/**
 * @def ONSENSOR(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define ONSENSOR(sensor_id)
/**
 * @def ONBUTTON(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define ONBUTTON(sensor_id)
/**
 * @def PAUSE
 * @brief 
 */
#define PAUSE
/**
 * @def PIN_TURNOUT(id,pin,description...)
 * @brief 
 * @param id 
 * @param pin 
 * @param description... 
 */
#define PIN_TURNOUT(id,pin,description...) 
/**
 * @def PRINT(msg)
 * @brief 
 * @param msg 
 */
#define PRINT(msg) 
/**
 * @def PARSE(msg)
 * @brief 
 * @param msg 
 */
#define PARSE(msg)
/**
 * @def PICKUP_STASH(id)
 * @brief 
 * @param id 
 */
#define PICKUP_STASH(id)
#ifndef DISABLE_PROG
/**
 * @def POM(cv,value)
 * @brief 
 * @param cv 
 * @param value 
 */
#define POM(cv,value)
#endif
/**
 * @def POWEROFF
 * @brief 
 */
#define POWEROFF
/**
 * @def POWERON
 * @brief 
 */
#define POWERON
/**
 * @def READ_LOCO
 * @brief 
 */
#define READ_LOCO 
/**
 * @def RED(signal_id)
 * @brief 
 * @param signal_id 
 */
#define RED(signal_id) 
/**
 * @def RESERVE(blockid)
 * @brief 
 * @param blockid 
 */
#define RESERVE(blockid) 
/**
 * @def RESET(pin,count...)
 * @brief 
 * @param pin 
 * @param count... 
 */
#define RESET(pin,count...) 
/**
 * @def RESUME
 * @brief 
 */
#define RESUME 
/**
 * @def RETURN
 * @brief 
 */
#define RETURN 
/**
 * @def REV(speed)
 * @brief 
 * @param speed 
 */
#define REV(speed) 
/**
 * @def ROTATE(turntable_id,position,activity)
 * @brief 
 * @param turntable_id 
 * @param position 
 * @param activity 
 */
#define ROTATE(turntable_id,position,activity)
/**
 * @def ROTATE_DCC(turntable_id,position)
 * @brief 
 * @param turntable_id 
 * @param position 
 */
#define ROTATE_DCC(turntable_id,position)
/**
 * @def ROSTER(cab,name,funcmap...)
 * @brief 
 * @param cab 
 * @param name 
 * @param funcmap... 
 */
#define ROSTER(cab,name,funcmap...)
/**
 * @def ROUTE(id,description)
 * @brief 
 * @param id 
 * @param description 
 */
#define ROUTE(id,description)
/**
 * @def ROUTE_ACTIVE(id)
 * @brief 
 * @param id 
 */
#define ROUTE_ACTIVE(id)
/**
 * @def ROUTE_INACTIVE(id)
 * @brief 
 * @param id 
 */
#define ROUTE_INACTIVE(id)
/**
 * @def ROUTE_HIDDEN(id)
 * @brief 
 * @param id 
 */
#define ROUTE_HIDDEN(id)
/**
 * @def ROUTE_DISABLED(id)
 * @brief 
 * @param id 
 */
#define ROUTE_DISABLED(id)
/**
 * @def ROUTE_CAPTION(id,caption)
 * @brief 
 * @param id 
 * @param caption 
 */
#define ROUTE_CAPTION(id,caption)
/**
 * @def SENDLOCO(cab,route)
 * @brief 
 * @param cab 
 * @param route 
 */
#define SENDLOCO(cab,route) 
/**
 * @def SEQUENCE(id)
 * @brief 
 * @param id 
 */
#define SEQUENCE(id) 
/**
 * @def SERIAL(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL(msg) 
/**
 * @def SERIAL1(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL1(msg) 
/**
 * @def SERIAL2(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL2(msg) 
/**
 * @def SERIAL3(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL3(msg) 
/**
 * @def SERIAL4(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL4(msg) 
/**
 * @def SERIAL5(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL5(msg) 
/**
 * @def SERIAL6(msg)
 * @brief 
 * @param msg 
 */
#define SERIAL6(msg) 
/**
 * @def SERVO(id,position,profile)
 * @brief 
 * @param id 
 * @param position 
 * @param profile 
 */
#define SERVO(id,position,profile) 
/**
 * @def SERVO2(id,position,duration)
 * @brief 
 * @param id 
 * @param position 
 * @param duration 
 */
#define SERVO2(id,position,duration) 
/**
 * @def SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
 * @brief 
 * @param vpin 
 * @param redpos 
 * @param amberpos 
 * @param greenpos 
 */
#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
/**
 * @def SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...)
 * @brief 
 * @param id 
 * @param pin 
 * @param activeAngle 
 * @param inactiveAngle 
 * @param profile 
 * @param description... 
 */
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) 
/**
 * @def SET(pin,count...)
 * @brief 
 * @param pin 
 * @param count... 
 */
#define SET(pin,count...) 
/**
 * @def SET_TRACK(track,mode)
 * @brief 
 * @param track 
 * @param mode 
 */
#define SET_TRACK(track,mode)
/**
 * @def SET_POWER(track,onoff)
 * @brief 
 * @param track 
 * @param onoff 
 */
#define SET_POWER(track,onoff)
/**
 * @def SETLOCO(loco)
 * @brief 
 * @param loco 
 */
#define SETLOCO(loco) 
/**
 * @def SETFREQ(freq)
 * @brief 
 * @param freq 
 */
#define SETFREQ(freq)
/**
 * @def SIGNAL(redpin,amberpin,greenpin)
 * @brief 
 * @param redpin 
 * @param amberpin 
 * @param greenpin 
 */
#define SIGNAL(redpin,amberpin,greenpin) 
/**
 * @def SIGNALH(redpin,amberpin,greenpin)
 * @brief 
 * @param redpin 
 * @param amberpin 
 * @param greenpin 
 */
#define SIGNALH(redpin,amberpin,greenpin) 
/**
 * @def SPEED(speed)
 * @brief 
 * @param speed 
 */
#define SPEED(speed) 
/**
 * @def START(route)
 * @brief 
 * @param route 
 */
#define START(route)
/**
 * @def STASH(id)
 * @brief 
 * @param id 
 */
#define STASH(id) 
/**
 * @def STEALTH(code...)
 * @brief 
 * @param code... 
 */
#define STEALTH(code...)
/**
 * @def STEALTH_GLOBAL(code...)
 * @brief 
 * @param code... 
 */
#define STEALTH_GLOBAL(code...)
/**
 * @def STOP
 * @brief 
 */
#define STOP 
/**
 * @def THROW(id)
 * @brief 
 * @param id 
 */
#define THROW(id)
/**
 * @def TOGGLE_TURNOUT(id)
 * @brief 
 * @param id 
 */
#define TOGGLE_TURNOUT(id)
/**
 * @def TT_ADDPOSITION(turntable_id,position,value,angle,description...)
 * @brief 
 * @param turntable_id 
 * @param position 
 * @param value 
 * @param angle 
 * @param description... 
 */
#define TT_ADDPOSITION(turntable_id,position,value,angle,description...)
/**
 * @def TURNOUT(id,addr,subaddr,description...)
 * @brief 
 * @param id 
 * @param addr 
 * @param subaddr 
 * @param description... 
 */
#define TURNOUT(id,addr,subaddr,description...) 
/**
 * @def TURNOUTL(id,addr,description...)
 * @brief 
 * @param id 
 * @param addr 
 * @param description... 
 */
#define TURNOUTL(id,addr,description...) 
/**
 * @def UNJOIN
 * @brief 
 */
#define UNJOIN 
/**
 * @def UNLATCH(sensor_id)
 * @brief 
 * @param sensor_id 
 */
#define UNLATCH(sensor_id) 
/**
 * @def VIRTUAL_SIGNAL(id)
 * @brief 
 * @param id 
 */
#define VIRTUAL_SIGNAL(id) 
/**
 * @def VIRTUAL_TURNOUT(id,description...)
 * @brief 
 * @param id 
 * @param description... 
 */
#define VIRTUAL_TURNOUT(id,description...) 
/**
 * @def WAITFOR(pin)
 * @brief 
 * @param pin 
 */
#define WAITFOR(pin)
#ifndef IO_NO_HAL
/**
 * @def WAITFORTT(turntable_id)
 * @brief 
 * @param turntable_id 
 */
#define WAITFORTT(turntable_id)
#endif
/**
 * @def WITHROTTLE(msg)
 * @brief 
 * @param msg 
 */
#define WITHROTTLE(msg)
/**
 * @def XFOFF(cab,func)
 * @brief 
 * @param cab 
 * @param func 
 */
#define XFOFF(cab,func)
/**
 * @def XFON(cab,func)
 * @brief 
 * @param cab 
 * @param func 
 */
#define XFON(cab,func)
/**
 * @def XFTOGGLE(cab,func)
 * @brief 
 * @param cab 
 * @param func 
 */
#define XFTOGGLE(cab,func)
/**
 * @def XFWD(cab,speed)
 * @brief 
 * @param cab 
 * @param speed 
 */
#define XFWD(cab,speed)
/**
 * @def XREV(cab,speed)
 * @brief 
 * @param cab 
 * @param speed 
 */
#define XREV(cab,speed)

#endif
