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

/* This file creates dummy entries for all the EXRAIL macros to allow
 * the EXRAIL Macro files to be parsed multiple times during compilation.
 * The actual definitions are created in EXRAILMacros.h according to the
 * current pass.
 *
 * Triple / Comments in this file are intended for the EXRAIL end user and will
 * be extracted to create documentation. Please maintain formatting.
 */

#define ACTIVATE(addr,subaddr)
///brief Send DCC Accessory Activate packet (gate on then off)
///param addr DCC short address of accessory
///param subaddr DCC sub address


#define ACTIVATEL(linearaddr)
///brief Send DCC Accessory Activate packet (gate on then off)
///param linearaddr DCC linear address of accessory

#define AFTER(vpin,timer...)
///brief Wait for sensor activated, then deactivated for given time
///param vpin Virtual Pin number of sensor
///param timer... optional wait in ms, default 500

#define AFTEROVERLOAD(track_id)
///brief Wait for overload to be resolved
///param track_id A..H

#define ALIAS(name,value...)
///brief defines a named numeric value.
///param name c++ variable name that can be used throughout the script
///param value...  if omitted, a large negative value is created automatically

#define AMBER(signal_id)
///brief Sets a signal to amber state
///see ONAMBER

#define ANOUT(vpin,value,param1,param2)
///brief Writes to the HAL analog output interface of a device driver.
///param vpin Virtual pin number of device
///param value basic analog value
///param param1 device dependent
///param param2 device dependent

#define AT(vpin)
///brief Wait until a sensor becomes active
///param vpin Virtual pin of sensor. Use negative value for sensors that are HIGH when activated

#define ASPECT(address,value)
///brief Sends a DCC aspect value to an accessory address. May also change status of a signal defined using this aspect.
///param address Linear DCC address of device
///param value Aspect value (Device dependent)

#define ATGTE(vpin,value)
///brief Wait for analog sensor to be greater than given value
///param vpin Analog pin number
///param value integer value to compare against

#define ATLT(vpin,value)
///brief Wait for analog sensor value to be less than given value
///param vpin Analog pin number
///param value integer value to compare against

#define ATTIMEOUT(vpin,timeout_ms)
///brief Wait for sensor active, with timeout. Use IFTIMEOUT to determine whether the AT was satisfied.
///see IFTIMEOUT
///param vpin Sensor pin number
///param timeout_ms Milliseconds to wait before timeout

#define AUTOMATION(sequence_id,description)
///brief Defines starting point of a sequence that will be shown as an Automation by
///      the throttles. Automations are started by the throttle handing over a loco id to be driven.
///param sequence_id Unique sequence id value
///param description (Quoted text) will be shown on throttle button

#define AUTOSTART
///brief A new task will be created starting from this point at Command Station startup

#define BLINK(vpin,onDuty,offDuty)
///brief Starts a blinking process for a vpin (typically a LED). Stop blink with SET or RESET.
///param vpin Pin to blink
///param onDuty Milliseconds with LED ON
///param offDuty Milliseconds with LED off

#define BROADCAST(msg)
///brief Send raw message text to all throttles using the DCC-EX protocol
///see WITHROTTLE
///param msg  Quoted message

#define BUILD_CONSIST(loco_id)
///brief Adds a loco to follow the current loco in a consist
///param loco_id may be negative to indicate loco facing backwards

#define BREAK_CONSIST
///brief Breaks up any consist involving the current loco

#define CALL(sequence_id)
///brief transfer control to another sequence with expectation to return
///see RETURN
///param sequence_id SEQUENCE to jump processing to, must terminate or RETURN

#define CLEAR_STASH(stash_id)
///brief Clears loco value stored in stash
///param stash_id which stash to clear.

#define CLEAR_ALL_STASH
///brief Clears all stashed loco values

#define CLEAR_ANY_STASH
///brief Clears loco value from all stash entries

#define CLOSE(turnout_id)
///brief Close turnout by id
///see THROW

#define CONFIGURE_SERVO(vpin,pos1,pos2,profile)
///brief Set up servo movement parameters for non-turnout
///param vpin must refer to a servo capable pin
///param pos1 SET position of servo
///param pos2 RESET position of servo
///param profile Movement profile (Instant, Fast, Medium, Slow, Bounce)

#define DCC_SIGNAL(signal_id,addr,subaddr)
///brief Define a DCC accessory signal with short address
///param signal_id Id used for all signal manipulation commands
///param addr DCC address
///param subaddr DCC subaddress

#define DCCX_SIGNAL(signal_id,redAspect,amberAspect,greenAspect)
///brief Define advanced DCC accessory signal with aspects
///param signal_id DCC Linear address AND Id used for all signal manipulation commands

#define DCC_TURNTABLE(turntable_id,home,description...)
///brief defines a Turntable device
///param homeAngle the angle of the home position, valid angles are 0 - 3600
///param description... Quoted text description of turntable

#define DEACTIVATE(addr,subaddr)
///brief Sends DCC Deactivate packet (gate on, gate off)
///param addr DCC accessory address
///param subaddr DCC accessory subaddress

#define DEACTIVATEL(addr)
///brief Sends DCC Deactivate packet (gate on, gate off)
///param addr DCC Linear accessory address

#define DELAY(delay_ms)
///brief Waits for given milliseconds delay (This is not blocking)
///param delay_mS Delay time in milliseconds

#define DELAYMINS(delay_minutes)
///brief Waits for given minutes delay (This is not blocking)

#define DELAYRANDOM(mindelay,maxdelay)
///brief Waits for random delay between min and max milliseconds (This is not blocking)
///param mindelay minimum delay in ms
///param maxdelay maximum delay in ms

#define DONE
///brief Stops task loco (if any) and terminates current task

#define DRIVE(analogpin)
///brief RESERVED do not use

#define ELSE
///brief introduces alternate processing path after any kind of IF
///see IF

#define ENDEXRAIL
///brief obsolete.. no longer needed. Does nothing.

#define ENDIF
///brief determines end of IF(any type)  block.
///      IF something ENDIF, or
///      IF something ELSE something ENDIF
///see IF

#define ENDTASK
///brief same as DONE
///see DONE

#define ESTOP
///brief Performs emergency stop on current task loco

#define ESTOPALL
///brief Performs emergency stop on all locos

#define EXRAIL
///brief obsolete.. no longer needed. Does nothing.

#define EXTT_TURNTABLE(id,vpin,home,description...)
///brief Defines the EX‑Turntable turntable/traverser object only, so you will need a separate HAL() statement for an EX‑Turntable device driver.
///param homeAngle  the angle of the home position, valid angles are 0 - 3600
///param quoted description...

#define FADE(vpin,value,ms)
///brief Modifies analog value slowly taking a given time
///param vpin Servo virtual pin number
///param value new target value
///param ms time to reach value

#define FOFF(func)
///brief Turns off current loco function
///see FON

#define FOLLOW(sequence_id)
///brief Task processing follows given route or sequence (Effectively a GoTo)

#define FON(func)
///brief Turn on current loco function
///see FOFF

#define FORGET
///brief Removes current loco from task and DCC reminders table.

#define FREE(token_id)
///brief Frees logical token
///see RESERVE
///param token_id 0..255

#define FREEALL
///brief Frees all logical tokens
///see RESERVE

#define FTOGGLE(func)
///brief Toggles function for current loco

#define FWD(speed)
///brief Instructs current loco to set DCC speed
///param speed 0..127   (1=ESTOP)

#define GREEN(signal_id)
///brief Sets signal to green state

#define HAL(haltype,params...)
///brief Defines VPIN mapping for specific hardware drivers
///param haltype driver name, normally device type
///param params... depend on driver.

#define HAL_IGNORE_DEFAULTS
///brief System will ignore default HAL device mappings

#define IF(vpin)
///brief Checks sensor state, If false jumps to matching nested ELSE or ENDIF
///param vpin  VPIN of sensor. Negative VPIN will invert sensor state.

#define IFAMBER(signal_id)
///brief Checks if signal is in AMBER state.
///see IF

#define IFCLOSED(turnout_id)
///brief Checks if given turnout is in close state
///see IF

#define IFGREEN(signal_id)
///brief Checks if given signal is in GREEN state
///see IF

#define IFGTE(vpin,value)
///brief Checks if analog vpin sensor >= value
///see IF

#define IFLOCO(loco_id)
///brief Checks if current task loco = loco_id
///see IF

#define IFLT(vpin,value)
///brief Checks if analog sensor < value
///see IF
///param vpin  Analog vpin of sensor

#define IFNOT(vpin)
///brief Inverse of IF
///see IF

#define IFRANDOM(percent)
///brief randomly satisfied IF at given percent probability
///see IF

#define IFRED(signal_id)
///brief Checks if given signal is in RED state
///see IF

#define IFSTASH(stash_id)
///brief Checks if given stash entry has a non-zero value
///see IF

#define IFSTASHED_HERE(stash_id)
///brief Checks if given stash entry has the current loco
///see IF

#define IFTHROWN(turnout_id)
///brief Checks if given turnout is in THROWN state
///see IF

#define IFRESERVE(token_id)
///brief Attempts to reserve token and if satisfiled the token remains reserved.
///see IF RESERVE FREE

#define IFTIMEOUT
///brief Checks TIMEOUT state after an AT/AFTER request with timeout value.
///see IF AT AFTER

#define IFTTPOSITION(turntable_id,position)
///brief Checks if Turntable is in given position
///see IF

#define IFRE(vpin,value)
///brief Checks external rotary encoder value
///param vpin of device driver for rotary encoder

#define IFBITMAP_ALL(vpin,mask)
///brief Checks if (vpin pseudo-analog value & mask) == mask.
///see IF
///param mask Binary mask applied to vpin value

#define IFBITMAP_ANY(vpin,mask)
///brief Checks if vpin pseudo-analog value & mask is non-zero
///see IF
///param mask Binary mask applied to vpin value

#define IFROUTE_ACTIVE(sequence_id)
///brief Checks if route is active
///see IF

#define IFROUTE_INACTIVE(sequence_id)
///brief Checks if route is inactive
///see IF

#define IFROUTE_HIDDEN(sequence_id)
///brief Checks if route is hidden
///see IF

#define IFROUTE_DISABLED(sequence_id)
///brief Checks if route is disabled
///see IF

#define INVERT_DIRECTION
///brief Marks current task so that FWD and REV commands are inverted.

#define JMRI_SENSOR(vpin,count...)
///brief Defines multiple JMRI `<s>` type sensor feedback definitions each with id matching vpin and INPUT_PULLUP
///param vpin first vpin number
///param count... Number of consecutive VPINS for which to create JMRI sensor feedbacks. Default 1.

#define JMRI_SENSOR_NOPULLUP(vpin,count...)
///brief Defines multiple JMRI `<s>` type sensor feedback definitions each with id matching vpin
///param vpin first vpin number
///param count... Number of consecutive VPINS for which to create JMRI sensor feedbacks. Default 1.

#define JOIN
///brief Switches PROG track to receive MAIN track DCC packets. (Drive on PROG track)

#define KILLALL
///brief Terminates all running EXRAIL tasks

#define LATCH(vpin)
///brief Make all AT/AFTER/IF see vpin as HIGH without checking hardware
///param vpin Must only be for VPINS 0..255

#define LCC(eventid)
///brief Issue event to LCC

#define LCCX(senderid,eventid)
///brief Issue LCC event while impersonating another sender

#define LCD(row,msg)
///brief Write message on row of default configured LCD/OLED
///see SCREEN
///param msg Quoted text

#define MOMENTUM(accel,decel...)
///brief sets momentum in ms per DCC 127 step for current loco.
///param accel Acceleration momentum
///param decel... Braking momentum. (=Acceleration if not given)

#define SCREEN(display,row,msg)
///brief Send message to external display handlers
///param display number, 0=local display, others are handled by external
///  displays which may have different display numbers on different devices.
///param msg Quoted text

#define LCN(msg)
///brief Reserved for LCN communication. Refer to their documentation.

#define MESSAGE(msg)
///brief Send a human readable message to all throttle users
///param msg Quoted text

#define MOVETT(turntable_id,steps,activity)
///brief Move Turntable to specific position
///see ROTATE
///param steps position to move to
///param activity see ROTATE

#define NEOPIXEL(vpin,r,g,b,count...)
///brief Set a NEOPIXEL vpin to a given red/green/blue colour
///param vpin VPIN of a pixel
///param r red component 0-255
///param g green component 0-255
///param b blue component 0-255
///param count... Number of consecutive pixels to set, Default 1.

#define NEOPIXEL_SIGNAL(vpin,redcolour,ambercolour,greencolour)
///brief Define a signal that uses a single multi colour pixel
///see NEORGB
///param vpin unique signal_id
///param redcolour  RGB colour use NEORGB(red,green,blue) to create values.

#define ACON(eventid)
///brief Send MERG CBUS ACON to Adapter

#define ACOF(eventid)
///brief Send MERG CBUS ACOF to Adapter

#define ONACON(eventid)
///brief Start task here when ACON for event received from MERG CBUS

#define ONACOF(eventid)
///brief Start task here when ACOF for event received from MERG CBUS

#define ONACTIVATE(addr,subaddr)
///brief Start task here when DCC Activate sent for short address

#define ONACTIVATEL(linear)
///brief Start task here when DCC Activate sent for linear address

#define ONAMBER(signal_id)
///brief Start task here when signal set to AMBER state

#define ONBLOCKENTER(block_id)
///brief Start task here when a loco enters a railcom block
///param block_id vpin associated to block by HAL(I2CRailcom..)

#define ONBLOCKEXIT(block_id)
///brief Start task here when a loco leaves a railcom block
///param block_id vpin associated to block by HAL(I2CRailcom..)

#define ONTIME(minute_in_day)
///brief Start task here when fastclock matches
///param minute_in_day (0..1439)

#define ONCLOCKTIME(hours,mins)
///brief Start task here when fastclock matches time

#define ONCLOCKMINS(mins)
///brief Start task here hourly when fastclock minutes matches

#define ONOVERLOAD(track_id)
///brief Start task here when given track goes into overload
///param track_id A..H

#define ONRAILSYNCON
///brief Start task here when the railsync (booster) input port get a valid DCC signal

#define ONRAILSYNCOFF
///brief Start task here when the railsync (booster) input port does not get a valid DCC signal any more

#define ONDEACTIVATE(addr,subaddr)
///brief Start task here when DCC deactivate packet sent

#define ONDEACTIVATEL(linear)
///brief Start task here when DCC deactivate sent to linear address

#define ONCLOSE(turnout_id)
///brief Start task here when turnout closed

#define ONLCC(sender,event)
///brief Start task here when LCC event arrives from sender

#define ONGREEN(signal_id)
///brief Start task here when signal set to GREEN state

#define ONRED(signal_id)
///brief Start task here when signal set to RED state

#define ONROTATE(turntable_id)
///brief Start task here when turntable is rotated

#define ONTHROW(turnout_id)
///brief Start task here when turnout is Thrown

#define ONCHANGE(vpin)
///brief Rotary encoder change starts task here (This is obscurely different from ONSENSOR which will be merged in a later release.)

#define ONSENSOR(vpin)
///brief Start task here when sensor changes state (debounced)

#define ONBITMAP(vpin)
///brief Start task here when bitmap sensor changes state (debounced)

#define ONBUTTON(vpin)
///brief Start task here when sensor changes HIGH to LOW.

#define PAUSE
///brief Pauses all EXRAIL tasks except the current one.
/// Other tasks ESTOP their locos until RESUME issued

#define PIN_TURNOUT(id,vpin,description...)
///brief Defines a turnout which operates on a single pin
///param description... Quoted text (shown to throttles) or HIDDEN

#define PRINT(msg)
///brief prints diagnostic message on USB serial
///param msg Quoted text

#define PARSE(msg)
///brief Executes `<>` command as if entered from serial
///param msg Quoted text, preferably including `<>`

#define PICKUP_STASH(stash_id)
///see STASH
///brief Loads stashed value into current task loco
///param stash_id position in stash where a loco id was previously saved.

#define POM(cv,value)
///brief Write value to cv on current tasks loco (Program on Main)

#define POWEROFF
///brief Powers off all tracks

#define POWERON
///brief Powers ON all tracks

#define RANDOM_CALL(sequence_id...)
///brief CALL to another sequence randomly chosen from list
///see CALL
///param sequence_id.. list of SEQUENCE that may be called, each must terminate or RETURN

#define RANDOM_FOLLOW(sequence_id...)
///brief jump to another sequence randomly chosen from list
///see FOLLOW
///param sequence_id.. list of SEQUENCE that may be followed

#define READ_LOCO
///brief Reads loco Id from prog track and sets currenmt task loco id.

#define RED(signal_id)
///brief sets signal to RED state

#define RESERVE(token_id)
///brief Waits for token for block. If not available immediately, current task loco is stopped.

#define RESERVE_NOESTOP(token_id)
///brief Reserves token for block without estopping the current task loco if token is already taken.

#define RESET(vpin,count...)
///brief Sets output pin LOW
///see SET
///param count... Number of consecutive pins, default 1

#define RESTORE_SPEED
///brief Resumes locos saved speed
///see SAVE_SPEED

#define RESUME
///brief Resumes PAUSEd tasks
///see PAUSE

#define RETURN
///brief Returns to CALL
///see CALL

#define REV(speed)
///brief Issues DCC speed packet for current loco in reverse.
///see FWD
///param speed  (0..127, 1=ESTOP)

#define ROTATE(turntable_id,position,activity)
///brief Rotates an EX-Turntable to a given position
///param activity .. One of:
/// - **Turn**: Rotate turntable, maintain phase
/// - **Turn_PInvert**: Rotate turntable, invert phase
/// - **Home**: Initiate homing
/// - **Calibrate**: Initiate calibration sequence
/// - **LED_On**: Turn LED on
/// - **LED_Slow**: Set LED to a slow blink
/// - **LED_Fast**: Set LED to a fast blink
/// - **LED_Off**: Turn LED off
/// - **Acc_On**: Turn accessory pin on
/// - **Acc_Off**: Turn accessory pin off

#define ROTATE_DCC(turntable_id,position_id)
///brief Rotates turntable to a given position using DCC commands

#define ROSTER(cab,name,funcmap...)
///brief Describes a loco roster entry visible to throttles
///param cab loco DCC address or 0 for default entry
///param name Quoted text
///param funcmap... Quoted text, optional list of function names separated by / character with momentary function names prefixed with an *.

#define ROUTE(sequence_id,description)
///brief Defines starting point of a sequence that will appear as a route on throttle buttons.
///param description Quoted text, throttle button caption.

#define ROUTE_ACTIVE(sequence_id)
///brief Tells throttle to display the route button as active
///param sequence_id of ROUTE/AUTOMATION

#define ROUTE_INACTIVE(sequence_id)
///brief Tells throttle to display the route button as inactive
///param sequence_id of ROUTE/AUTOMATION

#define ROUTE_HIDDEN(sequence_id)
///brief Tells throttle to hide the route button
///param sequence_id of ROUTE/AUTOMATION

#define ROUTE_DISABLED(sequence_id)
///brief Tells throttle to display the route button as disabled
///param sequence_id of ROUTE/AUTOMATION

#define ROUTE_CAPTION(sequence_id,caption)
///brief Tells throttle to change thr route button caption
///param sequence_id of ROUTE/AUTOMATION

#define SAVE_SPEED
///brief Resumes locos saved speed
///see RESTORE_SPEED

#define SENDLOCO(cab,sequence_id)
///brief Start a new task to drive the loco
///param cab loco to be driven
///param route sequence_id of route, automation or sequence to drive

#define SEQUENCE(sequence_id)
///brief Provides a unique label that can be used to call, follow or start.
///see CALL
///see FOLLOW
///see START

#define SERIAL(msg)
///brief Write direct to Serial output
///param msg Quoted text

#define SERIAL1(msg)
///brief Write direct to Serial1 output
///param msg Quoted text

#define SERIAL2(msg)
///brief Write direct to Serial2 output
///param msg Quoted text

#define SERIAL3(msg)
///brief Write direct to Serial3 output
///param msg Quoted text

#define SERIAL4(msg)
///brief Write direct to Serial4 output
///param msg Quoted text

#define SERIAL5(msg)
///brief Write direct to Serial5 output
///param msg Quoted text

#define SERIAL6(msg)
///brief Write direct to Serial6 output
///param msg Quoted text

#define SERVO(vpin,position,profile)
///brief Move servo to given position
///param vpin of servo
///param position  servo position (values are hardware dependent)
///param profile movement profile (Instant, Fast, Medium, Slow, Bounce)

#define SERVO2(vpin,position,duration)
///brief Move servo to given position taking time
///param vpin of servo
///param position  servo position (values are hardware dependent)
///param duration mS

#define SERVO_SIGNAL(vpin,redpos,amberpos,greenpos)
///brief Dedfine a servo based signal with 3 servo positions
///param vpin of servo, acts as signal_id
///param redpos servo position (values are hardware dependent)
///param amberpos servo position (values are hardware dependent)
///param greenpos servo position (values are hardware dependent)

#define SERVO_TURNOUT(turnout_id,vpin,activeAngle,inactiveAngle,profile,description...)
///brief Define a servo driven turnout
///param turnout_id used by THROW/CLOSE
///param vpin for servo
///param activeAngle servo position (values are hardware dependent)
///param inactiveAngle servo position (values are hardware dependent)
///param profile movement profile (Instant, Fast, Medium, Slow, Bounce)
///param description... Quoted text shown to throttles or HIDDEN keyword to hide turnout button

#define SET(vpin,count...)
///brief  Set pin HIGH
///see RESET
///param count...  Number of sequential vpins to set. Default 1.

#define SET_TRACK(track,mode)
///brief Set output track type
///param track A..H
///param mode NONE, MAIN, PROG, DC, EXT, BOOST, BOOST_INV, BOOST_AUTO, MAIN_INV, MAIN_AUTO, DC_INV, DCX

#define SET_POWER(track,onoff)
///brief Set track power mode
///param track A..H
///param onoff ON or OFF

#define SETLOCO(loco)
///brief Sets the loco being handled by the current task

#define SETFREQ(freq)
///brief Sets the DC track PWM frequency
///param freq Frequency is default 0, or 1..3

#define SIGNAL(redpin,amberpin,greenpin)
///brief Define a Signal with LOW=on leds
///see SIGNALH
///param redpin vpin for RED state, also acts as signal_id

#define SIGNALH(redpin,amberpin,greenpin)
///brief define a signal with HIGH=ON leds
///param redpin vpin for RED state, also acts as signal_id

#define SPEED(speed)
///brief Changes current tasks loco speed without changing direction
///param speed 0..127 (1=ESTOP)

#define START(sequence_id)
///brief Starts a new task at the given route/animation/sequence

#define START_SHARED(sequence_id)
///brief Starts a new task at the given route/animation/sequence and share current loco with it

#define START_SEND(sequence_id)
///brief Starts a new task at the given route/animation/sequence and send current loco to it. Remove loco from current task.

#define STASH(stash_id)
///brief saves current task's loco id in the stash array
///param stash_id  position in stash array to save loco id

#define STEALTH(code...)
///brief Allows for embedding raw C++ code in context of current task.
///param code... c++ code to be executed. This requires intimate understanding of the product architecture.

#define STEALTH_GLOBAL(code...)
///brief Allows for embedding raw c++ code out of context.
///param code...  c++ code to be defined. This requires intimate understanding of the product architecture.

#define STOP
///brief Same as SPEED(0)

#define THROW(turnout_id)
///brief Throws given turnout
///see CLOSE

#define TOGGLE_TURNOUT(turnout_id)
///brief Toggles given turnout

#define TT_ADDPOSITION(turntable_id,position_id,value,angle,description...)
///brief Defines a turntable track position
///param position_id each position is given an id
///param address DCC accessory address
///param angle Used only for throttles that may draw a visual representation of the turntable
///param description... quoted text or HIDDEN

#define TURNOUT(turnout_id,addr,subaddr,description...)
///brief Defines a DCC accessory turnout with legacy address
///param turnout_id to be used in THROW/CLOSE etc
///param addr DCC accessory address
///param subaddr DCC accessory subaddress
///param description... Quoted text or HIDDEN, appears on throttle buttons

#define TURNOUTL(turnout_id,addr,description...)
///brief Defines a DCC accessory turnout with linear address
///see TURNOUT
///param turnout_id to be used in THROW/CLOSE etc
///param addr DCC accessory linear address
///param description... Quoted text or HIDDEN, appears on throttle buttons

#define UNJOIN
///brief Disconnects PROG track from MAIN
///see JOIN

#define UNLATCH(vpin)
///brief removes latched on flag
///see LATCH
///param vpin (limited to 0..255)

#define VIRTUAL_SIGNAL(signal_id)
///brief Defines a virtual (no hardware) signal, use ONhandlers to simulate hardware
///see SIGNAL ONRED ONAMBER ONGREEN

#define VIRTUAL_TURNOUT(id,description...)
///brief Defines a virtual (no hardware) turnout, use ONhandlers to simulate hardware
///see TURNOUT ONCLOSE ONTHROW
///param description... quoted text or HIDDEN

#define BITMAP_AND(vpin1,mask)
///brief Performs a bitwise AND operation on the given vpin analog value and mask.
///param mask Binary mask to be ANDed with vpin1 value

#define BITMAP_INC(vpin)
///brief Increments pseudo analog value by 1

#define BITMAP_DEC(vpin)
///brief Decrements pseudo analog value by 1  (to zero)

#define BITMAP_OR(vpin1,mask)
///brief Performs a bitwise OR operation on the given vpin analog value and mask.
///param mask Binary mask to be ORed with vpin1 value

#define BITMAP_XOR(vpin1,mask)
///brief Performs a bitwise XOR operation on the given vpin analog value and mask.
///param mask Binary mask to be XORed with vpin1 value

#define WAITFOR(pin)
///brief Waits for completion of servo movement

#ifndef IO_NO_HAL

#define WAITFORTT(turntable_id)
///brief waits for completion of turntable movement
#endif

#define WAIT_WHILE_RED(signal_id)
///brief Keeps loco at speed 0 while signal is RED

#define WITHROTTLE(msg)
///brief Broadcasts a string in Withrottle protocol format to all throttles using this protocol.
///param msg quoted string

#define XFOFF(cab,func)
///brief Turns function off for given loco
///param func function number

#define XFON(cab,func)
///brief Turns function ON for given loco

#define XFTOGGLE(cab,func)
///brief Toggles function state for given loco

#define XFWD(cab,speed)
///brief Sends DCC speed to loco in forward direction
///param speed (0..127, 1=ESTOP)

#define XREV(cab,speed)
///brief Sends DCC speed to loco in reverse direction
///param speed (0..127, 1=ESTOP)

#define XPOM(cab,cv,value)
///brief Sends DCC speed to loco in reverse direction
///param cab loco id
///param cv  to be updated
///param value to be written to cv

#define XRESTORE_SPEED(cab)
///brief Resumes locos saved speed
///param cab loco id
///see XSAVE_SPEED

#define XSAVE_SPEED(cab)
///brief Resumes locos saved speed
///param cab loco id
///see XRESTORE_SPEED

#define ZTEST(command,testcode...)
///brief Developer Unit testing.  Do not use. 

#define ZTEST2(command,reply)
///brief Developer Unit testing.  Do not use.

#define ZTEST3(command,reply,testcode...)
///brief Developer Unit testing.  Do not use. 
