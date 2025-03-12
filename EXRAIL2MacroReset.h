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
#undef IFSTASH
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
#undef XPOM
#undef XREV
#undef XFWD

#ifndef RMFT2_UNDEF_ONLY
/**
 * @def ACTIVATE(addr,subaddr)
 * @brief Send DCC Accessory Activate packet (gate on then off)
 * @param addr DCC short address of accessory
 * @param subaddr DCC sub address
 */
#define ACTIVATE(addr,subaddr)
/**
 * @def ACTIVATEL(linearaddr)
 * @brief Send DCC Accessory Activate packet (gate on then off)
 * @param linearaddr DCC linear address of accessory
 */
#define ACTIVATEL(linearaddr)
/**
 * @def AFTER(vpin,timer...)
 * @brief Wait for sensor activated, then decativated for given time
 * @param vpin Virtual Pin number of sensor
 * @param timer... optional wait in mS, default 500
 */
#define AFTER(vpin,timer...)
/**
 * @def AFTEROVERLOAD(track_id)
 * @brief Wait for overload to be resolved
 * @param track_id A..H
 */
#define AFTEROVERLOAD(track_id)
/**
 * @def ALIAS(name,value...)
 * @brief defines a named numeric value.
 * @param name c++ variable name that can be used throighout the script
 * @param value...  if omitted, a large negative value is created automatically
 */
#define ALIAS(name,value...)
/**
 * @def AMBER(signal_id)
 * @brief Sets a signal to amber state
 * @see ONAMBER
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
 * @def AT(vpin)
 * @brief wait intil a sensor becomes active
 * @param vpin Virtual pin of sensor. Use negative value for sensors that are HIGH when activated 
 */
#define AT(vpin)
/**
 * @def ASPECT(address,value)
 * @brief Sends a DCC aspect value to an accessory address.
 *      May also change status of a signal defined using this aspect.
 * @param address Linear DCC address of device 
 * @param value Aspect value (Device dependent)
 */
#define ASPECT(address,value)
/**
 * @def ATGTE(vpin,value)
 * @brief Wait for analog sensor to be greater than given value
 * @param vpin Analog pin number 
 * @param value integer value to compare against
 */
#define ATGTE(vpin,value) 
/**
 * @def ATLT(vpin,value)
 * @brief Wait for analog sensor value to be less than given value
 * @param vpin Analog pin number 
 * @param value integer value to compare against
 */
#define ATLT(vpin,value) 
/**
 * @def ATTIMEOUT(vpin,timeout_ms)
 * @brief Wait for sensor active, with timeout. Use IFTIMEOUT to determine whether the AT was satisfied.
 * @see IFTIMEOUT  
 * @param vpin Sensor pin number 
 * @param timeout_ms Millseconds to wait before timeout
 */
#define ATTIMEOUT(vpin,timeout_ms)
/**
 * @def AUTOMATION(sequence_id,description)
 * @brief Defines starting point of a sequence that will be shown as an Automation by 
 * the throttles. Automations are started by the throttle handing over a loco id to be driven.
 * @param sequence_id Unique sequence id value
 * @param description (Quoted text) will be shown on throttle button
 */
#define AUTOMATION(sequence_id,description)
/**
 * @def AUTOSTART
 * @brief A new task will be created starting from this point at Command Station startup   
 */
#define AUTOSTART
/**
 * @def BLINK(vpin,onDuty,offDuty)
 * @brief Starts a blinking process for a vpin (typically a LED)
 * Stop blink with SET or RESET.
 * @param vpin Pin to blink
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
 * @def CALL(sequence_id)
 * @brief transfer control to another sequence with expectation to return
 * @see RETURN
 * @param sequence_id SEQUENCE to jump processing to, must terminate or RETURN
 *  
 */
#define CALL(sequence_id)
/**
 * @def CLEAR_STASH(stash_id)
 * @brief Clears loco value stored in stash  
 * @param stash_id which stash to clear.
 */
#define CLEAR_STASH(stash_id)
/**
 * @def CLEAR_ALL_STASH
 * @brief Clears all stashed loco values
 */
#define CLEAR_ALL_STASH
/**
* @def CLEAR_ANY_STASH
* @brief Clears loco value from all stash entries
*/
#define CLEAR_ANY_STASH
/**
 * @def CLOSE(turnout_id)
 * @brief Close turnout by id
 * @see THROW
 * @param turnout_id 
 */
#define CLOSE(turnout_id)
/**
 * @def CONFIGURE_SERVO(vpin,pos1,pos2,profile)
 * @brief setup servo movement parameters for non-turnout
 * @param vpin must refer to a servo capable pin
 * @param pos1 SET position of servo
 * @param pos2 RESET position of servo
 * @param profile Movement profile (Instant, Fast, Medium, Slow, Bounce)
 */
#define CONFIGURE_SERVO(vpin,pos1,pos2,profile) 
/**
 * @def DCC_SIGNAL(signal_id,addr,subaddr)
 * @brief Define a DCC accessory signal with short address
 * @param signal_id Id used for all signal manipulation commands
 * @param addr DCC address
 * @param subaddr DCC subaddress
 */
#define DCC_SIGNAL(signal_id,addr,subaddr)
/**
 * @def DCCX_SIGNAL(signal_id,redAspect,amberAspect,greenAspect)
 * @brief Dfeine advanced DCC accessory signal with aspects
 * @param signal_id DCC Linear address AND Id used for all signal manipulation commands
 * @param redAspect 
 * @param amberAspect 
 * @param greenAspect 
 */
#define DCCX_SIGNAL(signal_id,redAspect,amberAspect,greenAspect)
/**
 * @def DCC_TURNTABLE(turntable_id,homeAngle,description...)
 * @brief defines a Turntable device 
 * @param turntable_id 
 * @param homeAngle the angle of the home position, valid angles are 0 - 3600
 * @param description... Quoted text description of turntable
 */
#define DCC_TURNTABLE(turntable_id,home,description...)
/**
 * @def DEACTIVATE(addr,subaddr)
 * @brief Sends DCC Deactivate packet (gate on, gate off) 
 * @param addr DCC accessory address
 * @param subaddr DCC accessory subaddress
 */
#define DEACTIVATE(addr,subaddr)
/**
 * @def DEACTIVATEL(addr)
 * @brief Sends DCC Deactivate packet (gate on, gate off) 
 * @param addr DCC Linear accessory address
 */
#define DEACTIVATEL(addr)
/**
 * @def DELAY(delay_mS)
 * @brief Waits for given milliseconds delay (This is not blocking) 
 * @param delay_mS Delay time in milliseconds
 */
#define DELAY(delay_ms)
/**
 * @def DELAYMINS(delay_minutes)
 * @brief Waits for given minutes delay (This is not blocking)
 * @param delay_minutes 
 */
#define DELAYMINS(delay_minutes)
/**
 * @def DELAYRANDOM(mindelay,maxdelay)
 * @brief Waits for random delay between min and max milliseconds (This is not blocking)
 * @param mindelay minumum delay in mS
 * @param maxdelay maximum delay in mS
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
 * @see IF 
 */
#define ELSE
/**
 * @def ENDIF
 * @brief determines end of IF(any type)  block.
 * @see IF
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
 * @def ESTOPALL
 * @brief Performs emergency stop on all locos
 */
#define ESTOPALL
/**
 * @def EXTT_TURNTABLE(turntable_id,vpin,homeAngle,description...)
 * @brief This statement will create the EX‑Turntable turntable/traverser object only, so you will need a separate HAL() statement for an EX‑Turntable device driver.
 * @param turntable_id
 * @param vpin 
 * @param homeAngle  the angle of the home position, valid angles are 0 - 3600
 * @param description... 
 */
#define EXTT_TURNTABLE(id,vpin,home,description...)
/**
 * @def FADE(vpin,value,ms)
 * @brief Modifies analog value slowly taking a given time
 * @param vpin Servo virtual pin number 
 * @param value new target value
 * @param ms time to reach value
 */
#define FADE(vpin,value,ms)
/**
 * @def FOFF(func)
 * @brief Turns off current loco function
 * @see FON
 * @param func 
 */
#define FOFF(func)
/**
 * @def FOLLOW(sequence_id)
 * @brief Task processing follows given route or sequence (Effectively a GoTo)
 * @param sequence_id 
 */
#define FOLLOW(sequence_id) 
/**
 * @def FON(func)
 * @brief Turn on current loco function
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
 * @def FREE(token_id)
 * @brief Frees logical token 
 * @see RESERVE
 * @param token_id 0..255 
 */
#define FREE(token_id) 
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
 * @param params... depend on driver.
 */
#define HAL(haltype,params...)
/**
 * @def HAL_IGNORE_DEFAULTS
 * @brief System will ignore default HAL device mappings 
 */
#define HAL_IGNORE_DEFAULTS
/**
 * @def IF(vpin)
 * @brief Checks sensor state, If false jumps to matching nested ELSE or ENDIF
 * @param vpin  VPIN of sensor. Negative VPIN will invert sensor state. 
 */
#define IF(vpin) 
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
 * @def IFGTE(vpin,value)
 * @brief Checks if analog vpin sensor >= value
 * @see IF
 * @param vpin 
 * @param value 
 */
#define IFGTE(vpin,value) 
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
 * @param vpin  Analog vpin of sensor 
 * @param value 
 */
#define IFLT(vpin,value) 
/**
 * @def IFNOT(vpin)
 * @brief Inverse of IF
 * @see IF
 * @param vpin 
 */
#define IFNOT(vpin)
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
 * @def IFSTASH(stash_id)
 * @brief Checks if given stash entry has a non zero value
 * @see IF
 * @param stash_id 
 */
#define IFSTASH(stash_id) 
/**
 * @def IFTHROWN(turnout_id)
 * @brief Checks if given turnout is in THROWN state
 * @see IF
 * @param turnout_id 
 */
#define IFTHROWN(turnout_id) 
/**
 * @def IFRESERVE(token_id)
 * @brief Attempts to reserve token and if satisfiled the token remains reserved. 
 * @see IF RESERVE FREE
 * @param token_id
 */
#define IFRESERVE(token_id)
/**
 * @def IFTIMEOUT
 * @brief Checks TIMEOUT state after an AT/AFTER request with timeout value.
 * @see IF AT AFTER
 */
#define IFTIMEOUT
/**
 * @def IFTTPOSITION(turntable_id,position)
 * @brief Checks if Turntable is in given position
 * @see IF
 * @param turntable_id 
 * @param position 
 */
#define IFTTPOSITION(turntable_id,position)
/**
 * @def IFRE(vpin,value)
 * @brief Checks external rotary encoder value 
 * @param vpin of device driver for rotary encoder  
 * @param value 
 */
#define IFRE(vpin,value)
/**
 * @def INVERT_DIRECTION
 * @brief Marks current task so that FWD and REV commands are inverted.
 */
#define INVERT_DIRECTION 
/**
 * @def JMRI_SENSOR(vpin,count...)
 * @brief Defines multiple JMRI <s> type sensor feedback definitions each with id matching vpin
 * @param vpin first vpin number
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
 * @def LATCH(vpin)
 * @brief Make all AT/AFTER/IF see vpin as HIGH without checking hardware
 * @param vpin Must only be for VPINS 0..255
 */
#define LATCH(vpin)
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
 * @def MOMENTUM(accel,decel...)
 * @brief sets momentum in mS per DCC 127 step for curent loco. 
 * @param accel Acceleration momentum  
 * @param decel... Braking momantum. (=Acceleration of not given)
 */
#define MOMENTUM(accel,decel...)
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
 * @brief Reserved for LCN communication. Refer to their documentation.
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
 * @def MOVETT(turntable_id,steps,activity)
 * @brief Move Turntable to specific position
 * @see ROTATE
 * @param turntable_id
 * @param steps position to move to 
 * @param activity see ROTATE
 */
#define MOVETT(turntable_id,steps,activity)
/**
 * @def NEOPIXEL(vpin,r,g,b,count...)
 * @brief Set a NEOPIXEL vpin to a given red/green/blue colour
 * @param vpin VPIN of a pixel
 * @param r red component 0-255 
 * @param g green component 0-255
 * @param b blue component 0-255
 * @param count... Number of consecutive pixels to set, Default 1.
 */
#define NEOPIXEL(vpin,r,g,b,count...)
/**
 * @def NEOPIXEL_SIGNAL(signal_id,redcolour,ambercolour,greencolour)
 * @brief Define a signal that uses a single multi colour pixel
 * @see NEORGB 
 * @param vpin unique signal_id 
 * @param redcolour  RGB colour use NEORGB(red,green,blue) to create values.
 * @param ambercolour 
 * @param greencolour 
 */
#define NEOPIXEL_SIGNAL(vpin,redcolour,ambercolour,greencolour)
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
 * @brief Start task here when DCC Activate sent for linear address
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
 * @def OBLOCKENTER(block_id)
 * @brief Start task here when a loco enters a railcom block
 * @param block_id vpin associated to block by HAL(I2CRailcom..)  
 */
#define ONBLOCKENTER(block_id)
/**
 * @def OBLOCKEXIT(vpin)
 * @brief Start task here when a loco leaves a railcom block
 * @param block_id vpin associated to block by HAL(I2CRailcom..)  
 */
#define ONBLOCKEXIT(block_id)
/**
 * @def ONTIME(minute_in_day)
 * @brief Start task here when fastclock matches
 * @param minute_in_day (0..1439) 
 */
#define ONTIME(minute_in_day)
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
 * @brief Start task here when LCC event arrives from sender
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
 * @def ONCHANGE(vpin)
 * @brief Rotary encoder change starts task here (This is obscurely different from ONSENSOR which will be merged in a later release.) 
 * @param vpin 
 */
#define ONCHANGE(vpin)
/**
 * @def ONSENSOR(vpin)
 * @brief Start task here when sensor changes state (debounced)
 * @param vpin 
 */
#define ONSENSOR(vpin)
/**
 * @def ONBUTTON(vpin)
 * @brief Start task here when sensor changes HIGH to LOW. 
 * @param vpin 
 */
#define ONBUTTON(vpin)
/**
 * @def PAUSE
 * @brief Pauses all EXRAIL tasks except the curremnt one.
 * Other tasks ESTOP their locos until RESUME issued 
 */
#define PAUSE
/**
 * @def PIN_TURNOUT(turnout_id,vpin,description...)
 * @brief Defines a turnout which operates on a signle pin
 * @param turnout_id 
 * @param vpin 
 * @param description... Quoted text (shown to throttles) or HIDDEN 
 
 */
#define PIN_TURNOUT(id,vpin,description...) 
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
 * @def PICKUP_STASH(stash_id)
 * @see STASH
 * @brief Loads stashed value into current task loco
 * @param stash_id position in stash where a loco id was previously saved.
 */
#define PICKUP_STASH(stash_id)
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
 * @brief Reads loco Id from prog track and sets currenmt task loco id.
 */
#define READ_LOCO 
/**
 * @def RED(signal_id)
 * @brief sets signal to RED state 
 * @param signal_id 
 */
#define RED(signal_id) 
/**
 * @def RESERVE(token_id)
 * @brief Waits for token for block. If not available immediately, current task loco is stopped.
 * @param token_id 
 */
#define RESERVE(token_id) 
/**
 * @def RESET(vpin,count...)
 * @brief Sets output pin LOW
 * @see SET
 * @param vpin 
 * @param count... Number of consecutive pins, default 1
 */
#define RESET(vpin,count...) 
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
 * @def ROTATE(turntable_id,position_id,activity)
 * @brief Rotates an EX-Turntable to a given position
 * @param turntable_id 
 * @param position_id 
 * @param activity
 * - **Turn**: Rotate turntable, maintain phase
 * - **Turn_PInvert**: Rotate turntable, invert phase
 * - **Home**: Initiate homing
 * - **Calibrate**: Initiate calibration sequence
 * - **LED_On**: Turn LED on
 * - **LED_Slow**: Set LED to a slow blink
 * - **LED_Fast**: Set LED to a fast blink
 * - **LED_Off**: Turn LED off
 * - **Acc_On**: Turn accessory pin on
 * - **Acc_Off**: Turn accessory pin off
 */
#define ROTATE(turntable_id,position,activity)
/**
 * @def ROTATE_DCC(turntable_id,position_id)
 * @brief Rotates turntable to a given position using DCC commands
 * @param turntable_id 
 * @param position_id 
 */
#define ROTATE_DCC(turntable_id,position_id)
/**
 * @def ROSTER(cab,name,funcmap...)
 * @brief Describes a loco roster entry visible to throttles
 * @param cab loco DCC address or 0 for default entry
 * @param name Quoted text
 * @param funcmap... Quoted text, optional list of function names separated by / character with momentary function names prefixed with an *.
 * 
 */
#define ROSTER(cab,name,funcmap...)
/**
 * @def ROUTE(sequence_id,description)
 * @brief DEfines starting point of a sequence that will appear as a route on throttle buttons.
 * @param sequence_id 
 * @param description Quoted text, throttle button capotion. 
 */
#define ROUTE(sequence_id,description)
/**
 * @def ROUTE_ACTIVE(sequence_id)
 * @brief Tells throttle to display the route button as active
 * @param sequence_id of ROUTE/AUTOMATION 
 */
#define ROUTE_ACTIVE(sequence_id)
/**
 * @def ROUTE_INACTIVE(sequence_id)
 * @brief Tells throttle to display the route button as inactive
 * @param sequence_id of ROUTE/AUTOMATION
 */
#define ROUTE_INACTIVE(sequence_id)
/**
 * @def ROUTE_HIDDEN(sequence_id)
 * @brief Tells throttle to hide the route button
 * @param sequence_id of ROUTE/AUTOMATION
 */
#define ROUTE_HIDDEN(sequence_id)
/**
 * @def ROUTE_DISABLED(sequence_id)
 * @brief Tells throttle to display the route button as disabled
 * @param sequence_id of ROUTE/AUTOMATION 
 */
#define ROUTE_DISABLED(sequence_id)
/**
 * @def ROUTE_CAPTION(sequence_id,caption)
 * @brief Tells throttle to change thr route button caption
 * @param sequence_id of ROUTE/AUTOMATION 
 * @param caption 
 */
#define ROUTE_CAPTION(sequence_id,caption)
/**
 * @def SENDLOCO(cab,sequence_id)
 * @brief Start a new task to drive the loco 
 * @param cab loco to be driven 
 * @param route sequence_id of route, automation or sequence to drive
 */
#define SENDLOCO(cab,sequence_id) 
/**
 * @def SEQUENCE(sequence_id)
 * @brief Provides a unique label than can be used to call, follow or start. 
 * @see CALL
 * @see FOLLOW
 * @see START
 * @param sequence_id 
 */
#define SEQUENCE(sequence_id) 
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
 * @def SERVO(vpin,position,profile)
 * @brief Move servo to given position
 * @param vpin of servo
 * @param position  servo position (values are hardware dependent) 
 * @param profile movement profile (Instant, Fast, Medium, Slow, Bounce)
 */
#define SERVO(vpin,position,profile) 
/**
 * @def SERVO2(id,position,duration)
 * @brief Move servo to given position taking time 
 * @param vpin of servo
 * @param position  servo position (values are hardware dependent) 
 * @param duration mS
 */
#define SERVO2(vpin,position,duration) 
/**
 * @def SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
 * @brief Dedfine a servo based signal with 3 servo positions
 * @param vpin of servo, acts as signal_id
 * @param redpos servo position (values are hardware dependent) 
 * @param amberpos servo position (values are hardware dependent)
 * @param greenpos servo position (values are hardware dependent)
 */
#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
/**
 * @def SERVO_TURNOUT(turnout_id,vpin,activeAngle,inactiveAngle,profile,description...)
 * @brief Define a servo driven turnout
 * @param turnout_id used by THROW/CLOSE 
 * @param vpin for servo
 * @param activeAngle servo position (values are hardware dependent)
 * @param inactiveAngle servo position (values are hardware dependent)
 * @param profile movement profile (Instant, Fast, Medium, Slow, Bounce)
 * @param description... Quoted text shown to throttles or HIDDEN keyword to hide turnout button 
 */
#define SERVO_TURNOUT(turnout_id,vpin,activeAngle,inactiveAngle,profile,description...) 
/**
 * @def SET(vpin,count...)
 * @brief  Set pin HIGH
 * @see RESET  
 * @param vpin 
 * @param count...  Number of sequential vpins to set. Default 1. 
 */
#define SET(vpin,count...) 
/**
 * @def SET_TRACK(track,mode)
 * @brief Set output track type
 * @param track A..H
 * @param mode NONE, MAIN, PROG, DC, EXT, BOOST, BOOST_INV, BOOST_AUTO, MAIN_INV, MAIN_AUTO, DC_INV, DCX
 */
#define SET_TRACK(track,mode)
/**
 * @def SET_POWER(track,onoff)
 * @brief Set track power mode
 * @param track A..H
 * @param onoff ON or OFF
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
 * @param freq Frequency is default 0, or 1..3
 */
#define SETFREQ(freq)
/**
 * @def SIGNAL(redpin,amberpin,greenpin)
 * @brief Define a Signal with LOW=on leds 
 * @see SIGNALH  
 * @param redpin vpin for RED state, also acts as signal_id
 * @param amberpin 
 * @param greenpin 
 */
#define SIGNAL(redpin,amberpin,greenpin) 
/**
 * @def SIGNALH(redpin,amberpin,greenpin)
 * @brief define a signal with HIGH=ON leds 
 * @param redpin vpin for RED state, also acts as signal_id
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
 * @def START(sequence_id)
 * @brief Starts a new task at the given route/animation/sequence
 * @param sequence_id 
 */
#define START(sequence_id)
/**
 * @def STASH(stash_id)
 * @brief saves cuttent tasks loco id in the stash array
 * @param stash_id  position in stash array to save loco id 
 */
#define STASH(stash_id) 
/**
 * @def STEALTH(code...)
 * @brief Allows for embedding raw C++ code in context of current task.
 * @param code... c++ code to be executed. This requires intimate understanding of the product acrhitecture.
 */
#define STEALTH(code...)
/**
 * @def STEALTH_GLOBAL(code...)
 * @brief Allows for embedding raw c++ code out of context.
 * @param code...  c++ code to be defined. This requires intimate understanding of the product acrhitecture.
 */
#define STEALTH_GLOBAL(code...)
/**
 * @def STOP
 * @brief Same as SPEED(0)
 * @see SPEED
 */
#define STOP 
/**
 * @def THROW(turnout_id)
 * @brief Throws given turnout
 * @see CLOSE
 * @param turnout_id 
 */
#define THROW(turnout_id)
/**
 * @def TOGGLE_TURNOUT(turnout_id)
 * @brief Toggles given turnout
 * @param tuirnout_id 
 */
#define TOGGLE_TURNOUT(turnout_id)
/**
 * @def TT_ADDPOSITION(turntable_id,position_id,value,angle,description...)
 * @brief Defines a turntable track position
 * @param turntable_id 
 * @param position_id each position is given an id
 * @param address DCC accessory address 
 * @param angle Used only for throttles that may draw a visual representation of the turntable
 * @param description... quoted text or HIDDEN
 */
#define TT_ADDPOSITION(turntable_id,position_id,value,angle,description...)
/**
 * @def TURNOUT(turnout_id,addr,subaddr,description...)
 * @brief Defines a DCC accessory turnout with legacy address
 * @param turnout_id to be used in THROW/CLOSE etc  
 * @param addr DCC accessory address
 * @param subaddr DCC accessory subaddress
 * @param description... Quoted text or HIDDEN, appears on throttle buttons
 */
#define TURNOUT(turnout_id,addr,subaddr,description...) 
/**
 * @def TURNOUTL(turnout_id,addr,description...)
 * @brief Defines a DCC accessory turnout with inear address
 * @see TURNOUT
 * @param turnout_id to be used in THROW/CLOSE etc  
 * @param addr DCC accessory linear address
 * @param description... Quoted text or HIDDEN, appears on throttle buttons
 */
#define TURNOUTL(tirnout_id,addr,description...) 
/**
 * @def UNJOIN
 * @brief Disconnects PROG track from MAIN
 * @see JOIN
 */
#define UNJOIN 
/**
 * @def UNLATCH(vpin)
 * @brief removes latched on flag
 * @see LATCH 
 * @param vpin (limited to 0..255) 
 */
#define UNLATCH(vpin) 
/**
 * @def VIRTUAL_SIGNAL(signal_id)
 * @brief Defines a virtual (no hardware) signal, use ONhandlers to simulate hardware
 * @see SIGNAL ONRED ONAMBER ONGREEN
 * @param signal_id  
 */
#define VIRTUAL_SIGNAL(signal_id) 
/**
 * @def VIRTUAL_TURNOUT(turnout_id,description...)
 * @brief Defines a virtual (no hardware) turnout, use ONhandlers to simulate hardware
 * @see TURNOUT ONCLOSE ONTHROW
 * @param turnout_id 
 * @param description... quoted text or HIDDEN
 */
#define VIRTUAL_TURNOUT(id,description...) 
/**
 * @def WAITFOR(vpin)
 * @brief WAits for completion of servo movement
 * @param vpin 
 */
#define WAITFOR(pin)
#ifndef IO_NO_HAL
/**
 * @def WAITFORTT(turntable_id)
 * @brief waits for completion of turntable movement
 * @param turntable_id 
 */
#define WAITFORTT(turntable_id)
#endif
/**
 * @def WITHROTTLE(msg)
 * @brief Broadcasts a string in Withrottle protocol format to all throttles using this protocol. 
 * @param msg quoted string
 */
#define WITHROTTLE(msg)
/**
 * @def XFOFF(cab,func)
 * @brief Turns function off for given loco 
 * @param cab 
 * @param func function number
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
/**
 * @def XPOM(cab,cv,value)
 * @brief Sends DCC speed to loco in reverse direction
 * @param cab loco id
 * @param cv  to be updated
 * @param value to be written to cv
 */
#define XPOM(cab,cv,value)
#endif
