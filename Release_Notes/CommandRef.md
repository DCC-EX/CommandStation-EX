This file is being used to consolidate the command reference information. 

General points:
  - Commands below have a single character opcode and parameters.
  Even <JA> is actually read as <J A> 
  - Keyword parameters are shown in upper case but may be entered in mixed case.
  - value parameters are numeric.
  - [something]  indicates its optional.
  - Not all commands have a response, and not all responses come from the last commands that you have issued.
   
Startup status
<s>

Track power management
<1>
<1 MAIN|PROG|JOIN>
<0>
<0 MAIN|PROG>

Basic manual control
<t cab speed direction> 
<F cab function 1|0>
<!>
<T id 0|1|T|C>

DCC accessory control
<a address subaddress activate [onoff]>
<a linearaddress activate> 


Turnout definition
Note: Turnouts are best defined in myAutomation.h where a turnout description can also be provided ( refer to EXRAIL documentation) or by using these commands in a mySetup.h file. 

<T id SERVO vpin thrown closed profile>
<T id VPIN vpin>
<T id DCC addr subaddr>
<T id DCC linearaddr>


Outputs
<Z id activate>
<Z id vpin iflag>

Sensors 
<S id vpin pullup>

Decoder programming
<w cab cv value>
<b cab cv bit value>
<W cabid>
<W cv value>
<V cv value>
<V cv bit value>
<R>
<R cv>
<B cv bit value>
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

