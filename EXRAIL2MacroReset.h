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

// This file cleans and resets the RMFT2 Macros.
// It is used between passes to reduce complexity in RMFT2Macros.h
// DO NOT add an include guard to this file.

// Doxygen comments in this file are intended for the EXRAIL end user.

/**
 * @file EXRAIL2MacroReset.h
 * @mainpage EXRAIL Language Reference
 * 
 * @section introduction Introduction
 * EXRAIL - Extended Railroad Automation Instruction Language
 * 
 * This page is a reference to all EXRAIL commands available with EX-CommandStation.
 * 
 * - @ref ACTIVATE
 * - @ref ACTIVATEL
 */

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
#undef POM
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
 * @brief Send DCC Accessory Activate packet (gate on then off)
 * @param addr 
 * @param subaddr 
 */
#define ACTIVATE(addr,subaddr)
/**
 * @def ACTIVATEL(longaddr)
 * @brief Send DCC Accessory Activate packet (gate on then off)
 * @param longaddr 
 */
#define ACTIVATEL(longaddr)
/**
 * @def AFTER(sensor_id,timer...)
 * @brief Wait for sensor activated, then decativated for given time
 * @param sensor_id 
 * @param timer... optional wait in mS, default 500
 */
#define AFTER(sensor_id,timer...)
/**
 * @def AFTEROVERLOAD(track_id)
 * @brief Wait for overload to be resolved
 * @param track_id A..H
 */
#define AFTEROVERLOAD(track_id)
/**
 * @def ALIAS(name,value...)
 * @brief defines a named numeric value.
 * @param name 
 * @param value...  if omitted, a large negative value is created automatically
 */
#define ALIAS(name,value...)
/**
 * @def AMBER(signal_id)
 * @brief Sets a signal to amber state
 * @param signal_id 
 */
#define AMBER(signal_id)
/**
 * @def ANOUT(vpin,value,param1,param2)
 * @brief Writes to the HAL analog output interface of a device driver.
 *          Values and meanings of extra parameters depend on driver.
 * @param vpin Virtual pin number of device 
 * @param value basic analog value
 * @param param1 device dependent 
 * @param param2 device dependent
 */
#define ANOUT(vpin,value,param1,param2)
/**
 * @def AT(sensor_id)
 * @brief wait intil a sensor becomes active
 * @param sensor_id Use negative value for sensors that are HIGH when activated 
 */
#define AT(sensor_id)
/**
 * @def ASPECT(address,value)
 * @brief Sends a DCC aspect value to an accessory address.
 *      May also change status of a signal defined using this aspect.
 * @param address 
 * @param value 
 */
#define ASPECT(address,value)
/**
 * @def ATGTE(sensor_id,value)
 * @brief Wait for analog sensor to be greater than given value
 * @param sensor_id 
 * @param value 
 */
#define ATGTE(sensor_id,value) 
/**
 * @def ATLT(sensor_id,value)
 * @brief Wait for analog sensor value to be less than given value
 * @param sensor_id 
 * @param value 
 */
#define ATLT(sensor_id,value) 
/**
 * @def ATTIMEOUT(sensor_id,timeout_ms)
 * @brief Wait for sensor active, with timeout. Use IFTIMEOUT to determine whether the AT was satisfied.
 * @see IFTIMEOUT  
 * @param sensor_id 
 * @param timeout_ms 
 */
#define ATTIMEOUT(sensor_id,timeout_ms)
/**
 * @def AUTOMATION(id,description)
 * @brief Defies starting point of a sequence that will be shown as an Automation by 
 * the throttles. Automations are started by the throttle handing over a loco id to be driven.
 * @param id Unique sequence/route/automation id
 * @param description (Quoted text) will be shown on throttle button
 */
#define AUTOMATION(id,description)
/**
 * @def AUTOSTART
 * @brief A new task will be created starting from this point at Command Station startup   
 */
#define AUTOSTART
/**
 * @def BLINK(vpin,onDuty,offDuty)
 * @brief Starts a blinking process for a vpin (typically a LED)
 * Stop blink with SET or RESET.
 * @param vpin 
 * @param onDuty Milliseconds with LED ON
 * @param offDuty Milliseconds with LED off 
 */
#define BLINK(vpin,onDuty,offDuty)
/**
 * @def BROADCAST(msg)
 * @brief Send raw message text to all throttles using the DCC-EX protocol
 * @see WITHROTTLE
 * @param msg  Quoted message  
 */
#define BROADCAST(msg)
/**
 * @def CALL(route)
 * @brief transfer control to another sequence with expectation to return
 * @see RETURN
 * @param route Sequence id, sequence must terminate of RETURN
 *  
 */
#define CALL(route)
/**
 * @def CLEAR_STASH(id)
 * @brief Clears loco stash value  
 * @param id which stash to clear.
 */
#define CLEAR_STASH(id)
/**
 * @def CLEAR_ALL_STASH(id)
 * @brief ???????????????????????????????????????
 * @param id 
 */
#define CLEAR_ALL_STASH(id)
/**
 * @def CLOSE(id)
 * @brief Close turnout by id
 * @see THROW
 * @param id 
 */
#define CLOSE(id)
/**
 * @def CONFIGURE_SERVO(vpin,pos1,pos2,profile)
 * @brief setup servo movement parameters for non-turnout
 * @param vpin must refer to a servo capable pin
 * @param pos1 SET position of servo
 * @param pos2 RESET position of servo
 * @param profile Movement profile (????????)
 */
#define CONFIGURE_SERVO(vpin,pos1,pos2,profile) 
/**
 * @def DCC_SIGNAL(id,add,subaddr)
 * @brief Define a DCC accessory signal with short address
 * @param id Signal id used for all signal manipulation commands
 * @param add DCC address
 * @param subaddr DCC subaddress
 */
#define DCC_SIGNAL(id,add,subaddr)
/**
 * @def DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect)
 * @brief DEfine advanced DCC accessory signal with aspects
 * @param id Signal id used for all signal manipulation commands
 * @param redAspect 
 * @param amberAspect 
 * @param greenAspect 
 */
#define DCCX_SIGNAL(id,redAspect,amberAspect,greenAspect)
/**
 * @def DCC_TURNTABLE(id,home,description...)
 * @brief ??????????????????????????????????
 * @param id 
 * @param home 
 * @param description... 
 */
#define DCC_TURNTABLE(id,home,description...)
/**
 * @def DEACTIVATE(addr,subaddr)
 * @brief Sends DCC Deactivate packet (gate on, gate off) to short address
 * @param addr 
 * @param subaddr 
 */
#define DEACTIVATE(addr,subaddr)
/**
 * @def DEACTIVATEL(addr)
 * @brief Sends DCC Deactivate packet (gate on, gate off) to long address
 * @param addr 
 */
#define DEACTIVATEL(addr)
/**
 * @def DELAY(mindelay)
 * @brief Waits for given milliseconds delay (This is not blocking) 
 * @param mindelay mS
 */
#define DELAY(mindelay)
/**
 * @def DELAYMINS(mindelay)
 * @brief Waits for given minutes delay (This is not blocking)
 * @param mindelay 
 */
#define DELAYMINS(mindelay)
/**
 * @def DELAYRANDOM(mindelay,maxdelay)
 * @brief Waits for random delay between min and max milliseconds (This is not blocking)
 * @param mindelay mS
 * @param maxdelay mS
 */
#define DELAYRANDOM(mindelay,maxdelay) 
/**
 * @def DONE
 * @brief Stops task loco (if any) and terminates current task
 */
#define DONE
/**
 * @def DRIVE(analogpin)
 * @brief RESERVED do not use
 * @param analogpin 
 */
#define DRIVE(analogpin)
/**
 * @def ELSE
 * @brief introduces alternate processing path after any kind of IF 
 */
#define ELSE
/**
 * @def ENDEXRAIL
 * @brief Obsolete, has no effect.
 */
#define ENDEXRAIL 
/**
 * @def ENDIF
 * @brief determines end of IF(any type)  block.
 * IF something ENDIF, or    
 * IF something ELSE something ENDIF  
 */
#define ENDIF  
/**
 * @def ENDTASK
 * @brief same as DONE
 * @see DONE
 */
#define ENDTASK
/**
 * @def ESTOP
 * @brief Performs emergency stop on current task loco
 */
#define ESTOP 
/**
 * @def EXRAIL
 * @brief OBSOLETE ignored
 */
#define EXRAIL
/**
 * @def EXTT_TURNTABLE(id,vpin,home,description...)
 * @brief ??????????????????????
 * @param id 
 * @param vpin 
 * @param home 
 * @param description... 
 */
#define EXTT_TURNTABLE(id,vpin,home,description...)
/**
 * @def FADE(pin,value,ms)
 * @brief Modifies analog value slowly taking a given time
 * @param pin 
 * @param value new target value
 * @param ms time to reach value
 */
#define FADE(pin,value,ms)
/**
 * @def FOFF(func)
 * @brief Turns off loco function for current loco
 * @see FON
 * @param func 
 */
#define FOFF(func)
/**
 * @def FOLLOW(route)
 * @brief Task processing follows given route or sequence (Effectively a GoTo)
 * @param route 
 */
#define FOLLOW(route) 
/**
 * @def FON(func)
 * @brief Turn on current loc finction
 * @see FOFF
 * @param func 
 */
#define FON(func)
/**
 * @def FORGET
 * @brief Removes current loco from task and DCC reminders table.
 */
#define FORGET
/**
 * @def FREE(blockid)
 * @brief Frees logical token for given block.
 * @see RESERVE
 * @param blockid 0..255 
 */
#define FREE(blockid) 
/**
 * @def FTOGGLE(func)
 * @brief Toggles function for current loco
 * @param func 
 */
#define FTOGGLE(func)
/**
 * @def FWD(speed)
 * @brief Instructs current loco to set DCC speed
 * @param speed 0..127   (1=ESTOP)
 */
#define FWD(speed) 
/**
 * @def GREEN(signal_id)
 * @brief Sets signal to green state 
 * @param signal_id 
 */
#define GREEN(signal_id)
/**
 * @def HAL(haltype,params...)
 * @brief Defines VPIN mapping for specific hardware drivers
 * @param haltype driver name, normally device type
 * @param params... depend on driver
 */
#define HAL(haltype,params...)
/**
 * @def HAL_IGNORE_DEFAULTS
 * @brief System will ignore default HAL settings 
 */
#define HAL_IGNORE_DEFAULTS
/**
 * @def IF(sensor_id)
 * @brief Checks sensor state, If false jumps to matching nested ELSE or ENDIF
 * @param sensor_id  VPIN of sensor. Negative VPIM will invert sensor state. 
 */
#define IF(sensor_id) 
/**
 * @def IFAMBER(signal_id)
 * @brief Checks if signal is in AMBER state. 
 * @see IF
 * @param signal_id 
 */
#define IFAMBER(signal_id)
/**
 * @def IFCLOSED(turnout_id)
 * @brief Checks if given turnout is in close state
 * @see IF
 * @param turnout_id 
 */
#define IFCLOSED(turnout_id) 
/**
 * @def IFGREEN(signal_id)
 * @brief Checks if given signal is in GREEN state
 * @see IF
 * @param signal_id 
 */
#define IFGREEN(signal_id)
/**
 * @def IFGTE(sensor_id,value)
 * @brief Checks if analog sensor >= value
 * @see IF
 * @param sensor_id 
 * @param value 
 */
#define IFGTE(sensor_id,value) 
/**
 * @def IFLOCO(loco_id)
 * @brief Checks if current task loco = loco_id
 * @see IF
 * @param loco_id 
 */
#define IFLOCO(loco_id)
/**
 * @def IFLT(sensor_id,value)
 * @brief Checks if analog sensor < value
 * @see IF
 * @param sensor_id 
 * @param value 
 */
#define IFLT(sensor_id,value) 
/**
 * @def IFNOT(sensor_id)
 * @brief Inverse of IF
 * @see IF
 * @param sensor_id 
 */
#define IFNOT(sensor_id)
/**
 * @def IFRANDOM(percent)
 * @brief randomly satisfield IF at given percent probability
 * @see IF
 * @param percent 
 */
#define IFRANDOM(percent)
/**
 * @def IFRED(signal_id)
 * @brief Checks if given signal is in RED state
 * @see IF
 * @param signal_id 
 */
#define IFRED(signal_id)
/**
 * @def IFTHROWN(turnout_id)
 * @brief Checks if given turnout is in THROWN state
 * @see IF
 * @param turnout_id 
 */
#define IFTHROWN(turnout_id) 
/**
 * @def IFRESERVE(block)
 * @brief Agttempts to reserve block token and if satisfiled the block remains reserved. 
 * @see IF 
 * @param block 
 */
#define IFRESERVE(block)
/**
 * @def IFTIMEOUT
 * @brief Checks TIMEOUT state after an AT/AFTER request with timeout value.
 * @see IF
 */
#define IFTIMEOUT
/**
 * @def IFTTPOSITION(turntable_id,position)
 * @brief Checks if GTurntable is in given position
 * @see IF
 * @param turntable_id 
 * @param position 
 */
#define IFTTPOSITION(turntable_id,position)
/**
 * @def IFRE(sensor_id,value)
 * @brief ????????????????????????????????????????
 * @param sensor_id 
 * @param value 
 */
#define IFRE(sensor_id,value)
/**
 * @def INVERT_DIRECTION
 * @brief Marks current task so that FWD and REV commands are inverted.
 */
#define INVERT_DIRECTION 
/**
 * @def JMRI_SENSOR(vpin,count...)
 * @brief DEfines multiple JMRI <s> type sensor feedback definitions each with id matching vpin
 * @param vpin 
 * @param count... Number of consecutine VPINS for which to create JMRI sensor feedbacks. Default 1. 
 */
#define JMRI_SENSOR(vpin,count...)
/**
 * @def JOIN
 * @brief Switches PROG track to receive MAIN track DCC packets. (Drive on PROG track)
 */
#define JOIN 
/**
 * @def KILLALL
 * @brief Tertminates all running EXRAIL tasks
 */
#define KILLALL
/**
 * @def LATCH(sensor_id)
 * @brief Make all AT/AFTER/IF see sensor active without checking hardware
 * @param sensor_id Must only be for VPINS 0..255
 */
#define LATCH(sensor_id)
/**
 * @def LCC(eventid)
 * @brief Issue event to LCC 
 * @param eventid 
 */
#define LCC(eventid) 
/**
 * @def LCCX(senderid,eventid)
 * @brief Issue LCC event while impersonating another sender 
 * @param senderid 
 * @param eventid 
 */
#define LCCX(senderid,eventid) 
/**
 * @def LCD(row,msg)
 * @brief Write message on row of default configured LCD/OLED
 * @see SCREEN 
 * @param row 
 * @param msg Quoted text
 */
#define LCD(row,msg)
/**
 * @def SCREEN(display,row,msg)
 * @brief Send message to external display hadlers 
 * @param display number, 0=local display, others are handled by external
 *  displays which may have different display numbers on different devices. 
 * @param row 
 * @param msg Quoted text 
 */
#define SCREEN(display,row,msg)
/**
 * @def LCN(msg)
 * @brief ??????
 * @param msg 
 */
#define LCN(msg) 
/**
 * @def MESSAGE(msg)
 * @brief Send a human readable message to all throttle users
 * @param msg Quoted text 
 */
#define MESSAGE(msg)
/**
 * @def MOVETT(id,steps,activity)
 * @brief ???????????????????
 * @param id 
 * @param steps 
 * @param activity 
 */
#define MOVETT(id,steps,activity)
/**
 * @def NEOPIXEL(id,r,g,b,count...)
 * @brief Set a NEOPIXEL vpin to a given red/green/blue colour
 * @param id VPIN of a pixel
 * @param r red component 0-255 
 * @param g green component 0-255
 * @param b blue component 0-255
 * @param count... Number of consecutive pixels to set, Default 1.
 */
#define NEOPIXEL(id,r,g,b,count...)
/**
 * @def NEOPIXEL_SIGNAL(sigid,redcolour,ambercolour,greencolour)
 * @brief Define a signal that uses a single multi colour pixel
 * @param sigid unique signal id
 * @param redcolour  RGB colour 
 * @param ambercolour 
 * @param greencolour 
 * Use NeoRGB(red,green,blue) to create values for redcolour etc above.
 */
#define NEOPIXEL_SIGNAL(sigid,redcolour,ambercolour,greencolour)
/**
 * @def ACON(eventid)
 * @brief Send MERG CBUS ACON to Adapter
 * @param eventid 
 */
#define ACON(eventid)
/**
 * @def ACOF(eventid)
 * @brief Send MERG CBUS ACOF to Adapter
 * @param eventid 
 */
#define ACOF(eventid)
/**
 * @def ONACON(eventid)
 * @brief Start task here when ACON for event receied from MERG CBUS
 * @param eventid 
 */
#define ONACON(eventid)
/**
 * @def ONACOF(eventid)
 * @brief Start task here when ACOF for event receied from MERG CBUS
 * @param eventid 
 */
#define ONACOF(eventid)
/**
 * @def ONACTIVATE(addr,subaddr)
 * @brief Start task here when DCC Activate sent for short address
 * @param addr 
 * @param subaddr 
 */
#define ONACTIVATE(addr,subaddr)
/**
 * @def ONACTIVATEL(linear)
 * @brief Start task here when DCC Activate sent for long address
 * @param linear 
 */
#define ONACTIVATEL(linear)
/**
 * @def ONAMBER(signal_id)
 * @brief Start task here when signal set to AMBER state
 * @param signal_id 
 */
#define ONAMBER(signal_id) 
/**
 * @def ONTIME(value)
 * @brief Start task here when fastclock mins in day=value
 * @param value 
 */
#define ONTIME(value)
/**
 * @def ONCLOCKTIME(hours,mins)
 * @brief Start task here when fastclock matches time
 * @param hours 
 * @param mins 
 */
#define ONCLOCKTIME(hours,mins)
/**
 * @def ONCLOCKMINS(mins)
 * @brief Start task here hourly when fastclock minutes matches 
 * @param mins 
 */
#define ONCLOCKMINS(mins)
/**
 * @def ONOVERLOAD(track_id)
 * @brief Start task here when given track goes into overload
 * @param track_id A..H
 */
#define ONOVERLOAD(track_id)
/**
 * @def ONDEACTIVATE(addr,subaddr)
 * @brief Start task here when DCC deactivate packet sent
 * @param addr 
 * @param subaddr 
 */
#define ONDEACTIVATE(addr,subaddr)
/**
 * @def ONDEACTIVATEL(linear)
 * @brief Start task here when DCC deactivate sent to linear address
 * @param linear 
 */
#define ONDEACTIVATEL(linear) 
/**
 * @def ONCLOSE(turnout_id)
 * @brief Start task here when turnout closed
 * @param turnout_id 
 */
#define ONCLOSE(turnout_id)
/**
 * @def ONLCC(sender,event)
 * @brief ??????????????????
 * @param sender 
 * @param event 
 */
#define ONLCC(sender,event)
/**
 * @def ONGREEN(signal_id)
 * @brief Start task here when signal set to GREEN state 
 * @param signal_id 
 */
#define ONGREEN(signal_id) 
/**
 * @def ONRED(signal_id)
 * @brief Start task here when signal set to RED state 
 * @param signal_id 
 */
#define ONRED(signal_id)
/**
 * @def ONROTATE(turntable_id)
 * @brief Start task here when turntable is rotated
 * @param turntable_id 
 */
#define ONROTATE(turntable_id)
/**
 * @def ONTHROW(turnout_id)
 * @brief Start task here when turnout is Thrown
 * @param turnout_id 
 */
#define ONTHROW(turnout_id) 
/**
 * @def ONCHANGE(sensor_id)
 * @brief ???????????????????
 * @param sensor_id 
 */
#define ONCHANGE(sensor_id)
/**
 * @def ONSENSOR(sensor_id)
 * @brief Start task here when sensor changes state (debounced)
 * @param sensor_id 
 */
#define ONSENSOR(sensor_id)
/**
 * @def ONBUTTON(sensor_id)
 * @brief Start task here when sensor changes HIGH to LOW. 
 * @param sensor_id 
 */
#define ONBUTTON(sensor_id)
/**
 * @def PAUSE
 * @brief Pauses all EXRAIL tasks except the curremnt one.
 * Other tasks ESTOP their locos until RESUME issued 
 */
#define PAUSE
/**
 * @def PIN_TURNOUT(id,pin,description...)
 * @brief Defines a tirnout which operates on a signle pin
 * @param id 
 * @param pin 
 * @param description... Quoted text (shown to throttles) or HIDDEN 
 
 */
#define PIN_TURNOUT(id,pin,description...) 
/**
 * @def PRINT(msg)
 * @brief prints diagnostic message on USB serial 
 * @param msg Quoted text
 */
#define PRINT(msg) 
/**
 * @def PARSE(msg)
 * @brief Executes <> command as if entered from serial
 * @param msg Quoted text, preferably including <>
 */
#define PARSE(msg)
/**
 * @def PICKUP_STASH(id)
 * @brief Loads stashed value into current task loco
 * @param id position in stash where a loco id was previously saved.
 */
#define PICKUP_STASH(id)
/**
 * @def POM(cv,value)
 * @brief Write value to cv on current tasks loco (Program on Main) 
 * @param cv 
 * @param value 
 */
#define POM(cv,value)
/**
 * @def POWEROFF
 * @brief Powers off all tracks
 */
#define POWEROFF
/**
 * @def POWERON
 * @brief Powers ON all tracks 
 */
#define POWERON
/**
 * @def READ_LOCO
 * @brief Reads loco Id from prog traqck and sets currenmt task loco id.
 */
#define READ_LOCO 
/**
 * @def RED(signal_id)
 * @brief sets signal to RED state 
 * @param signal_id 
 */
#define RED(signal_id) 
/**
 * @def RESERVE(blockid)
 * @brief Waits for token for block. If not available immediately, current task loco is stopped.
 * @param blockid 
 */
#define RESERVE(blockid) 
/**
 * @def RESET(pin,count...)
 * @brief Sets output puin LOW
 * @param pin 
 * @param count... Number of consecutive pins, default 1
 */
#define RESET(pin,count...) 
/**
 * @def RESUME
 * @brief Resumes PAUSEd tasks 
 * @see PAUSE
 */
#define RESUME 
/**
 * @def RETURN
 * @brief Returns to CALL 
 * @see CALL
 */
#define RETURN 
/**
 * @def REV(speed)
 * @brief Issues DCC speed packet for current loco in reverse.
 * @see FWD
 * @param speed  (0..127, 1=ESTOP) 
 */
#define REV(speed) 
/**
 * @def ROTATE(turntable_id,position,activity)
 * @brief ????
 * @param turntable_id 
 * @param position 
 * @param activity 
 */
#define ROTATE(turntable_id,position,activity)
/**
 * @def ROTATE_DCC(turntable_id,position)
 * @brief ????
 * @param turntable_id 
 * @param position 
 */
#define ROTATE_DCC(turntable_id,position)
/**
 * @def ROSTER(cab,name,funcmap...)
 * @brief Describes a loco roster entry visible to throttles
 * @param cab loco DCC address or 0 for default entry
 * @param name Quoted text
 * @param funcmap... Quoted text, optional list of function names separated by / character with momentary fuinctin names prefixed with an *.
 * 
 */
#define ROSTER(cab,name,funcmap...)
/**
 * @def ROUTE(id,description)
 * @brief DEfines starting point of a sequence that will appear as a route on throttle buttons.
 * @param id 
 * @param description Quoted text, throttle button capotion. 
 */
#define ROUTE(id,description)
/**
 * @def ROUTE_ACTIVE(id)
 * @brief Tells throttle to display the route button as active
 * @param id 
 */
#define ROUTE_ACTIVE(id)
/**
 * @def ROUTE_INACTIVE(id)
 * @brief Tells throttle to display the route button as inactive
 * @param id 
 */
#define ROUTE_INACTIVE(id)
/**
 * @def ROUTE_HIDDEN(id)
 * @brief Tells throttle to hide the route button
 * @param id 
 */
#define ROUTE_HIDDEN(id)
/**
 * @def ROUTE_DISABLED(id)
 * @brief Tells throttle to display the route button as disabled
 * @param id 
 */
#define ROUTE_DISABLED(id)
/**
 * @def ROUTE_CAPTION(id,caption)
 * @brief Tells throttle to change thr route button caption
 * @param id 
 * @param caption 
 */
#define ROUTE_CAPTION(id,caption)
/**
 * @def SENDLOCO(cab,route)
 * @brief Start a new task to drive the loco 
 * @param cab loco to be driven 
 * @param route id of route/automation or sequence to drive
 */
#define SENDLOCO(cab,route) 
/**
 * @def SEQUENCE(id)
 * @brief Provides a unique label than can be used to call, follow or start. 
 * @see CALL
 * @see FOLLOW
 * @see START
 * @param id 
 */
#define SEQUENCE(id) 
/**
 * @def SERIAL(msg)
 * @brief Write direct to Serial output
 * @param msg Quoted text 
 */
#define SERIAL(msg) 
/**
 * @def SERIAL1(msg)
 * @brief Write direct to Serial1 output
 * @param msg Quoted text 
 */
#define SERIAL1(msg) 
/**
 * @def SERIAL2(msg)
 * @brief Write direct to Serial2 output
 * @param msg Quoted text 
 */
#define SERIAL2(msg) 
/**
 * @def SERIAL3(msg)
 * @brief Write direct to Serial3 output
 * @param msg Quoted text 
*/
#define SERIAL3(msg) 
/**
 * @def SERIAL4(msg)
 * @brief Write direct to Serial4 output
 * @param msg Quoted text 
 */
#define SERIAL4(msg) 
/**
 * @def SERIAL5(msg)
 * @brief Write direct to Serial5 output
 * @param msg Quoted text 
 */
#define SERIAL5(msg) 
/**
 * @def SERIAL6(msg)
 * @brief Write direct to Serial6 output
 * @param msg Quoted text 
 */
#define SERIAL6(msg) 
/**
 * @def SERVO(id,position,profile)
 * @brief Move servo to given position
 * @param id VPIN of servo
 * @param position 
 * @param profile ?????????? names ???????????
 */
#define SERVO(id,position,profile) 
/**
 * @def SERVO2(id,position,duration)
 * @brief Move servo to given position taking time 
 * @param id 
 * @param position 
 * @param duration mS
 */
#define SERVO2(id,position,duration) 
/**
 * @def SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
 * @brief Dedfine a servo based signal with 3 servo positions
 * @param vpin 
 * @param redpos 
 * @param amberpos 
 * @param greenpos 
 */
#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
/**
 * @def SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...)
 * @brief Define a servo driven turnout
 * @param id used by THROW/CLOSE 
 * @param pin VPIN for servo
 * @param activeAngle 
 * @param inactiveAngle 
 * @param profile ??????
 * @param description... Quoted text shown to throttles or HIDDEN keyword to hide turnout button 
 */
#define SERVO_TURNOUT(id,pin,activeAngle,inactiveAngle,profile,description...) 
/**
 * @def SET(pin,count...)
 * @brief  Set VPIN HIGH  
 * @param pin 
 * @param count...  Number of sequential vpins to set. Default 1. 
 */
#define SET(pin,count...) 
/**
 * @def SET_TRACK(track,mode)
 * @brief Set output track type
 * @param track A..H
 * @param mode ???names???
 */
#define SET_TRACK(track,mode)
/**
 * @def SET_POWER(track,onoff)
 * @brief Set track power mode
 * @param track A..H
 * @param onoff ??? values ???
 */
#define SET_POWER(track,onoff)
/**
 * @def SETLOCO(loco)
 * @brief Sets the loco being handled by the current task
 * @param loco 
 */
#define SETLOCO(loco) 
/**
 * @def SETFREQ(freq)
 * @brief Sets the DC track PWM frequency 
 * @param freq ??????????? values ??????
 */
#define SETFREQ(freq)
/**
 * @def SIGNAL(redpin,amberpin,greenpin)
 * @brief Define a Signal with LOW=on leds (is that common annode???)
 * @see SIGNALH  
 * @param redpin 
 * @param amberpin 
 * @param greenpin 
 */
#define SIGNAL(redpin,amberpin,greenpin) 
/**
 * @def SIGNALH(redpin,amberpin,greenpin)
 * @brief define a signal with HIGH=ON leds 
 * @param redpin 
 * @param amberpin 
 * @param greenpin 
 */
#define SIGNALH(redpin,amberpin,greenpin) 
/**
 * @def SPEED(speed)
 * @brief Changes current tasks loco speed without changing direction
 * @param speed 0..127 (1=ESTOP)
 */
#define SPEED(speed) 
/**
 * @def START(route)
 * @brief Starts a new task at the given route/animation/sequence
 * @param route 
 */
#define START(route)
/**
 * @def STASH(id)
 * @brief saves cuttent tasks loco id in the stash array
 * @param id 
 */
#define STASH(id) 
/**
 * @def STEALTH(code...)
 * @brief Allows for embedding raw C++ code in context of current task.
 * @param code... 
 */
#define STEALTH(code...)
/**
 * @def STEALTH_GLOBAL(code...)
 * @brief Allows for embedding raw c++ code out of context.
 * @param code... 
 */
#define STEALTH_GLOBAL(code...)
/**
 * @def STOP
 * @brief Same as SPEED(0)
 */
#define STOP 
/**
 * @def THROW(id)
 * @brief Throws given turnout
 * @param id 
 */
#define THROW(id)
/**
 * @def TOGGLE_TURNOUT(id)
 * @brief Toggles given turnout
 * @param id 
 */
#define TOGGLE_TURNOUT(id)
/**
 * @def TT_ADDPOSITION(turntable_id,position,value,angle,description...)
 * @brief Defines a turntable track position
 * @param turntable_id 
 * @param position ??????????
 * @param value 
 * @param angle 
 * @param description... 
 */
#define TT_ADDPOSITION(turntable_id,position,value,angle,description...)
/**
 * @def TURNOUT(id,addr,subaddr,description...)
 * @brief Defines a DCC accessory turnout with legacy address
 * @param id 
 * @param addr 
 * @param subaddr 
 * @param description... Quoted text or HIDDEN, appears on throttle buttons
 */
#define TURNOUT(id,addr,subaddr,description...) 
/**
 * @def TURNOUTL(id,addr,description...)
 * @brief Defines a DCC accessory turnout with inear address
 * @param 
 * @param id 
 * @param addr 
 * @param description... 
 */
#define TURNOUTL(id,addr,description...) 
/**
 * @def UNJOIN
 * @brief Disconnects PROG track from MAIN
 * @see JOIN
 */
#define UNJOIN 
/**
 * @def UNLATCH(sensor_id)
 * @brief removes latched on flag
 * @see LATCH 
 * @param sensor_id 
 */
#define UNLATCH(sensor_id) 
/**
 * @def VIRTUAL_SIGNAL(id)
 * @brief Defines a virtual (no hardware) signal
 * @param id 
 */
#define VIRTUAL_SIGNAL(id) 
/**
 * @def VIRTUAL_TURNOUT(id,description...)
 * @brief Defines a virtual (no hardware) turnout
 * @param id 
 * @param description... 
 */
#define VIRTUAL_TURNOUT(id,description...) 
/**
 * @def WAITFOR(pin)
 * @brief ???????????????????
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
 * @brief Broadcasts a string in Withrottle protocol format to all throttles using this protocol. 
 * @param msg 
 */
#define WITHROTTLE(msg)
/**
 * @def XFOFF(cab,func)
 * @brief Turns function off for given loco 
 * @param cab 
 * @param func 
 */
#define XFOFF(cab,func)
/**
 * @def XFON(cab,func)
 * @brief Turns function ON for given loco
 * @param cab 
 * @param func 
 */
#define XFON(cab,func)
/**
 * @def XFTOGGLE(cab,func)
 * @brief Toggles function state for given loco
 * @param cab 
 * @param func 
 */
#define XFTOGGLE(cab,func)
/**
 * @def XFWD(cab,speed)
 * @brief Sends DCC speed to loco in forward direction
 * @param cab 
 * @param speed (0..127, 1=ESTOP)
 */
#define XFWD(cab,speed)
/**
 * @def XREV(cab,speed)
 * @brief Sends DCC speed to loco in reverse direction
 * @param cab 
 * @param speed (0..127, 1=ESTOP)
 */
#define XREV(cab,speed)

#endif
