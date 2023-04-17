This file is being used to consolidate the command reference information. 

General points:
  - Commands below have a single character opcode and parameters.
  Even <JA> is actually read as <J A> 
  - Keyword parameters are shown in upper case but may be entered in mixed case.
  - value parameters are decimal numeric (unless otherwise noted)
  - [something]  indicates its optional.
  - Not all commands have a response, and broadcasts mean that not all responses come from the last commands that you have issued.
   
Startup status
<s> Return status like
    <iDCC-EX V-4.2.22 / MEGA / STANDARD_MOTOR_SHIELD G-devel-202302281422Z>
    also returns defined turnout list:
    <H id 1|0>   1=thrown

Track power management. After power commands a power state is broadcast to all throttles.

<1>                Power on all
<1 MAIN|PROG|JOIN> Power on MAIN or PROG track
<1 JOIN>           Power on MAIN and PROG track but send main track data on both.
<0>                Power off all tracks 
<0 MAIN|PROG>      Power off main or prog track

Basic manual loco control
<t locoid speed direction>  Throttle loco. 
    speed in JMRI-form  (-1=ESTOP, 0=STOP, 1..126 = DCC speeds 2..127)
    direction 1=forward, 0=reverse
    For response see broadcast <l>

<F locoid function 1|0>     Set loco function 1=ON, 0-OFF
    For response see broadcast <l>

<!>  emergency stop all locos
<T id 0|1|T|C>  Control turnout id, 0=C=Closed, 1=T=Thrown
       response broadcast <H id 0|1>


DCC accessory control
<a address subaddress activate [onoff]>
<a linearaddress activate> 


Turnout definition
Note: Turnouts are best defined in myAutomation.h where a turnout description can also be provided ( refer to EXRAIL documentation) or by using these commands in a mySetup.h file. 

<T id SERVO vpin thrown closed profile>
<T id VPIN vpin>
<T id DCC addr subaddr>
<T id DCC linearaddr>
   Valid commands respond with <O>

Direct pin manipulation (replaces <Z commands, no predefinition required)
<z vpin>     Set pin HIGH
<z -vpin>    Set pin LOW
<z vpin value> Set pin analog value
<z vpin value profile> Set pin analog with profile
<z vpin value profile duration> set pin analog with profile and value


Sensors (Used by JMRI, not required by EXRAIL)
<S id vpin pullup> define a sensor to be monitored.
   Responses <Q id> and <q id> as sensor changes  

Decoder programming - main track
<w cab cv value> POM write value to cv on loco
<b cab cv bit value> POM write bit to cv on loco

Decoder Programming - prog track
<W cabid>  Clear consist and write new cab id (includes long/short settings)
           Responds <W cabid> or <W -1> for error
<W cv value> Write value to cv

<V cv predictedValue> Read cv value, much faster if prediction is correct. 
<V cv bit predictedValue>      Read CV bit

<R>         Read drive-away loco id. (May be a consist id)
<D ACK ON|OFF>
<D ACK LIMIT|MIN|MAX|RETRY value>
<D PROGBOOST>

Advanced DCC control
<M  packet.... >
<P  packet ...>
<f map1 map2 [map3]>
<#>
<->
<- cabid>
<D CABS>
<D SPEED28>
<D SPEED128>


EEPROM commands
These commands exist for
backwards JMRI compatibility. 
You are strongly discouraged from maintaining your configuration settings in EEPROM.    
<E>
<e>
<D EEPROM>
<T>
<T id>
<S>
<S id>
<Z>
<Z id>

Diagnostic commands
<D CMD ON|OFF>
<D WIFI ON|OFF>
<D ETHERNET ON|OFF>
<D WIT ON|OFF>
<D LCN ON|OFF>
<D EXRAIL ON|OFF>
<D RESET>
<D SERVO|ANOUT vpin position [profile]>
<D ANIN vpin>
<D HAL SHOW>
<D HAL RESET>
<+ cmd>
<+>
<Q>

User defined filter commands
<U ....>
<u ....>

Track Management
<=>
<= track DCC|PROG|OFF>
<= track DC|DCX cabid>
<JG>
<JI>


Turntable interface
<D TT vpin steps [activity]>

Fast clock interface
<JC>
<JC mins rate>


Advanced Throttle access to features
<t cab>
<JA>
<JA id>
<JR>
<JR id>
<JT>
<JT id>

*******************
EXRAIL Commands
*******************

</>
</PAUSE>
</RESUME>
</START cab sequence>
</START sequence>
</KILL taskid>
</KILL ALL>
</RESERVE|FREE blockid>
</LATCH|UNLATCH latchid>
</RED|AMBER|GREEN signalid>

Obsolete commands/formats
<c>
<t ignored cab speed direction> 
<T id vpin thrown closed>
<T id addr subaddr>
<B cv bit value obsolete obsolete>
<R cv obsolete obsolete>
<W cv value obsolete obsolete>
<R cv>            V command is much faster if prediction is correct.
<B cv bit value>  V command is much faster if prediction is correct.
<Z id vpin active> (use <z)  Define an output pin that JMRI can set by id 
<Z id activate>    (use <z)  Activate an output pin by id


Broadcast responses
Note: broadcasts are sent to all throttles when appropriate (usually because something has changed)

<p0>
<p1>
<p1 MAIN|PROG|JOIN>

<l cab slot dccspeed functionmap>
<H id 1|0>
<jC mmmm speed>

Diagnostic responses
These are not meant to be software readable. They contain diagnostic information for programmers to identify issues.
<X> 
<*  ... *>

