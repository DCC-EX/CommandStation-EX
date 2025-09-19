/*
 *  © 2022 Paul M Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Mike S
 *  © 2021-2025 Herb Morton
 *  © 2020-2023 Harald Barth
 *  © 2020-2021 M Steve Todd
 *  © 2020-2021 Fred Decker
 *  © 2020-2025 Chris Harlow
 *  © 2022 Colin Murdoch
 *  All rights reserved.
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

/*
List of single character OPCODEs in use for reference.

When determining a new OPCODE for a new feature, refer to this list as the source of truth.

Once a new OPCODE is decided upon, update this list.

  Character, Usage
  /, |EX-R| interactive commands
  -, Remove from reminder table
  =, |TM| configuration
  !, Emergency stop
  @, Reserved for future use - LCD messages to JMRI
  #, Request number of supported cabs/locos; heartbeat
  +, WiFi AT commands
  ?, Reserved for future use
  0, Track power off
  1, Track power on
  a, DCC accessory control
  A, DCC extended accessory control
  b, Write CV bit on main
  B, Write CV bit
  c, Request current command
  C, configure the CS
  d,
  D, Diagnostic commands
  e, Erase EEPROM
  E, Store configuration in EEPROM
  f, Loco decoder function control (deprecated)
  F, Loco decoder function control
  g,
  G,
  h,
  H, Turnout state broadcast
  i, Server details string
  I, Turntable object command, control, and broadcast
  j, Throttle responses
  J, Throttle queries
  k, Block exit  (Railcom)
  K, Block enter (Railcom)
  l, Loco speedbyte/function map broadcast
  L, Reserved for LCC interface (implemented in EXRAIL)
  m, message to throttles (broadcast output) 
  m, set momentum  
  M, Write DCC packet
  n, Reserved for SensorCam
  N, Reserved for Sensorcam 
  o, Neopixel driver (see also IO_NeoPixel.h)
  O, Output broadcast
  p, Broadcast power state
  P, Write DCC packet
  q, Sensor deactivated
  Q, Sensor activated
  r, Broadcast address read on programming track
  R, Read CVs
  s, Display status
  S, Sensor configuration
  t, Cab/loco update command
  T, Turnout configuration/control
  u, Reserved for user commands
  U, Reserved for user commands
  v,
  V, Verify CVs
  w, Write CV on main
  W, Write CV
  x,
  X, Invalid command response
  y, 
  Y, Output broadcast
  z, Direct output
  Z, Output configuration/control
*/
/*
Each ZZ macro matches a command opcode and its parameters.
Paramters in UPPER case are matched as keywords, parameters in lower case are values provided by the user.
Its important to recognise that if the same opcode has more than one match with the same length, you must match the
keywprds before picking up user values. 
e.g. 
 ZZ(X,value1,value2)
 ZZ(X,SET,value1)  This will never be matched.


Use of the CHECK() macro validates a condition to be true.
If the condition is false an error is generated, resulting in an <X> reply.
Commonly known parameters such as loco, cv bitvalue etc are range checked automatically.
The REPLY( format, ...) macro sends a formatted string to the stream. 
 
These macros are included into the DCCEXParser::execute function so
  stream, ringStream and other DCCEXParser variables are available in context. */

 

ZZBEGIN
ZZ(#) // Request number of simultaneously supported locos
        REPLY( "<# %d>\n", MAX_LOCOS)
ZZ(!)   // Emergency stop all locos
        DCC::estopAll(); 
ZZ(t,loco) // Request loco status
        CommandDistributor::broadcastLoco(DCC::lookupSpeedTable(loco,false));
ZZ(t,loco,tspeed,direction) // Set throttle speed(0..127) and direction (0=reverse, 1=fwd) 
        CHECK(setThrottle(loco,tspeed,direction)) 
ZZ(t,ignore,loco,tspeed,direction) // (Deprecated) Set throttle speed and direction
        CHECK(setThrottle(loco,tspeed,direction)) 
ZZ(f,loco,byte1)  // (Deprecated use F) Set loco function group 
    switch ( byte1 & 0b11110000) { // 1111 0000
        case 0b11100000: // 111x xxxx Function group 1 F0..F4
        case 0b11110000: 
          // Shuffle bits from order F0 F4 F3 F2 F1 to F4 F3 F2 F1 F0
          return (funcmap(loco, (byte1 << 1 & 0x1e) | (byte1 >> 4 & 0x01), 0, 4));
        case 0b10110000: // 1011 xxxx Function group 2 F5..F8
          return (funcmap(loco, byte1, 5, 8));
        case 0b10100000: // 1010 xxxx Function group 3 F9..F12
          return (funcmap(loco, byte1, 9, 12));
        default: 
          CHECK(false,Invalid function group)
    }
ZZ(f,loco,group,byte2) // (Deprecated use F) Set loco function group 
   if (group == 222)  return (funcmap(loco, byte2, 13, 20));
   if (group == 223)  return (funcmap(loco, byte2, 21, 28));
   CHECK(false,Invalid function group)
ZZ(T)  // List all turnouts
        Turnout::printAll(stream); // will <X> if none found
ZZ(T,id) // Delete turnout
        CHECK(Turnout::remove(id)) 
ZZ(T,id,X) // List turnout details
        auto tt=Turnout::get(id); CHECK(tt)  tt->print(stream);     
ZZ(T,id,T) // Throw Turnout
        Turnout::setClosed(id, false);     
ZZ(T,id,C) // Close turnout#
        Turnout::setClosed(id, true);      
ZZ(T,id,value) // Close (value=0) ot Throw turnout
        Turnout::setClosed(id, value==0);      
ZZ(T,id,SERVO,vpin,closedValue,thrownValue) // Create Servo turnout  
        CHECK(ServoTurnout::create(id, (VPIN)vpin, (uint16_t)closedValue, (uint16_t)thrownValue, 1)) 
ZZ(T,id,VPIN,vpin)  // Create pin turnout
        CHECK(VpinTurnout::create(id, vpin)) 
ZZ(T,id,DCC,addr,subadd) // Create DCC turnout 
        CHECK(DCCTurnout::create(id, addr, subadd)) 
ZZ(T,id,DCC,linearAddr)  // Create DCC turnout
        CHECK(DCCTurnout::create(id, (linearAddr-1)/4+1, (linearAddr-1)%4)) 
ZZ(T,id,addr,subadd) // Create DCC turnout
        CHECK(DCCTurnout::create(id, addr, subadd)) 
ZZ(T,id,vpin,closedValue,thrownValue) // Create SERVO turnout
        CHECK(ServoTurnout::create(id, (VPIN)vpin, (uint16_t)closedValue, (uint16_t)thrownValue, 1))
ZZ(S,id,vpin,pullup)  // Create Sensor
        CHECK(Sensor::create(id,vpin,pullup)) 
ZZ(S,id) // Delete sensor
        CHECK(Sensor::remove(id))
ZZ(S)   // List sensors
        for (auto *tt = Sensor::firstSensor; tt; tt = tt->nextSensor) {
                REPLY("<Q %d %d %d>\n", tt->data.snum, tt->data.pin, tt->data.pullUp)
        }           
ZZ(J,M) // List stash values
        Stash::list(stream);
ZZ(J,M,stash_id) // get stash value
        Stash::list(stream, stash_id);
ZZ(J,M,CLEAR,ALL) // Clear all stash values
        Stash::clearAll(); 
ZZ(J,M,CLEAR,stash_id) // Clear given stash
        Stash::clear(stash_id); 
ZZ(J,M,stashId,locoId) // Set stash value
        Stash::set(stashId,locoId); 
ZZ(J,M,CLEAR,ANY,locoId) // Clear all stash entries that contain locoId
        Stash::clearAny(locoId);
ZZ(J,C) // get fastclock time
        REPLY("<jC %d>\n", CommandDistributor::retClockTime())
ZZ(J,C,mmmm,nn) // Set fastclock time
        CommandDistributor::setClockTime(mmmm, nn, 1);    
ZZ(J,G) // FReport gauge limits 
        TrackManager::reportGauges(stream);       
ZZ(J,I) // Report currents 
        TrackManager::reportCurrent(stream);      
// TODO... Ask @Ash zz(J,L,display,row) // Direct current displays to LCS/OLED
//         TrackManager::reportCurrentLCD(display,row);   // Track power status     
ZZ(J,A) // List Routes
        REPLY( "<jA>\n") 
ZZ(J,R) // List Roster
        REPLY("<jR") 
        #ifdef EXRAIL_ACTIVE
        SENDFLASHLIST(stream,RMFT2::rosterIdList) 
        #endif                
        REPLY(">\n");      
#ifdef EXRAIL_ACTIVE
ZZ(J,R,id) // Get roster for loco
        auto rosterName= RMFT2::getRosterName(id);
        if (!rosterName) rosterName=F("");
        auto functionNames= RMFT2::getRosterFunctions(id);
        if (!functionNames) functionNames=RMFT2::getRosterFunctions(0);
        if (!functionNames) functionNames=F("");
        REPLY("<jR %d \"%S\" \"%S\">\n",id, rosterName, functionNames)
#endif
ZZ(J,T) // Get turnout list 
        REPLY("<jT")
        for ( auto t=Turnout::first(); t; t=t->next()) if (!t->isHidden())  REPLY(" %d",t->getId()) 
        REPLY(">\n");
ZZ(J,T,id) // Get turnout state and description
        auto t=Turnout::get(id);
        if (!t || t->isHidden()) { REPLY("<jT %d X>\n",id) return true; }
        const FSH *tdesc=nullptr;
        #ifdef EXRAIL_ACTIVE
        tdesc = RMFT2::getTurnoutDescription(id);
        #endif
        if (!tdesc) tdesc = F("");
        REPLY("<jT %d %c \"%S\">\n",id,t->isThrown()?'T':'C',tdesc)
ZZ(z,vpin)  // Set pin. HIGH iv vpin positive, LOW if vpin negative  
        IODevice::write(vpin,(vpin>0)?HIGH:LOW);
ZZ(z,vpin,analog,profile,duration) // Change analog value over duration (Fade or servo move)
        IODevice::writeAnalogue(vpin,analog,profile,duration);
ZZ(z,vpin,analog,profile) // Write analog device using profile number (Fade or servo movement)
        IODevice::writeAnalogue(vpin,analog,profile,0);
ZZ(z,vpin,analog) // Write analog device value
        IODevice::writeAnalogue(vpin,analog,0,0);
     
// ==========================
// Turntable - no support if no HAL
// <I> - list all
// <I id> - broadcast type and current position
// <I id DCC> - create DCC - This is TBA
// <I id steps> - operate (DCC)
// <I id steps activity> - operate (EXTT)
// <I id ADD position value> - add position
// <I id EXTT i2caddress vpin home> - create EXTT

ZZ(I)  // List all turntables
        return Turntable::printAll(stream);
ZZ(I,id)  // Broadcast turntable type and current position    
        auto tto = Turntable::get(id);
        CHECK(tto,Turntable not found)
        REPLY("<I %d %d>\n", tto->isEXTT(), tto->getPosition())
ZZ(I,id,position) // Rotate a DCC turntable
        auto tto = Turntable::get(id);
        CHECK(tto,Turntable not found)
        CHECK(!tto->isEXTT(),Turntable type incorrect)
        CHECK(tto->setPosition(id,position))

ZZ(I,id,DCC,home) // Create DCC turntable
        CHECK(home >=0 && home <= 3600)
        auto tto = Turntable::get(id);
        CHECK(!tto,Turntable already exists)
        CHECK(DCCTurntable::create(id)) 
        tto = Turntable::get(id);
        CHECK(tto)
        tto->addPosition(0, 0, home);
        REPLY("<I>\n")

ZZ(I,id,position,activity) // Rotate an EXTT turntable
        auto tto = Turntable::get(id); 
        CHECK(tto,Turntable not found)
        CHECK(tto->isEXTT(), Turntable wrong type)
        CHECK(tto->setPosition(id, position,activity))
    
ZZ(I,id,EXTT,vpin,home) // Create an EXTT turntable
        auto tto = Turntable::get(id);
        CHECK(!tto,Turntable already exists)
        CHECK(home >= 0 && home <= 3600)
        CHECK(EXTTTurntable::create(id, (VPIN)vpin))
        tto = Turntable::get(id);
        tto->addPosition(0, 0, home);
        REPLY("<I>\n")

ZZ(I,id,ADD,position,value,angle) // Add turntable position
        auto tto = Turntable::get(id);
        CHECK(tto,Turntable not found)
        CHECK(position <= 48 && angle >=0  && angle <= 3600)
        tto->addPosition(id,value,angle);
        REPLY("<I>\n")

 ZZ(Q) // List all sensors 
        Sensor::printAll(stream);

 ZZ(s) // Command station status
        REPLY("<iDCC-EX V-" VERSION " / " ARDUINO_TYPE " / %S G-" GITHUB_SHA ">\n", DCC::getMotorShieldName())
        CommandDistributor::broadcastPower(); // <s> is the only "get power status" command we have
        Turnout::printAll(stream); //send all Turnout states
        Sensor::printAll(stream);  //send all Sensor  states
               

#ifndef DISABLE_EEPROM
    ZZ(E) // STORE EPROM
        EEStore::store();
        REPLY("<e %d %d %d>\n", EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs)
    ZZ(e) // CLEAR EPROM
        EEStore::clear();
        REPLY("<O>\n")
#endif

ZZ(Z) // List Output definitions 
        bool gotone = false;
        for (auto *tt = Output::firstOutput; tt ; tt = tt->nextOutput) {
          gotone = true;
          REPLY("<Y %d %d %d %d>\n",tt->data.id, tt->data.pin, tt->data.flags, tt->data.active)    
        }
        CHECK(gotone,No Outputs found)

ZZ(Z,id,pin,iflag) // Create Output
        CHECK(id > 0 && iflag >= 0 && iflag <= 7 )
        CHECK(Output::create(id,pin,iflag, 1))
        REPLY("<O>\n")
        ZZ(Z,id,active) // Set output 
        auto o = Output::get(id);
        CHECK(o,Output not found)
        o->activate(active);
        REPLY("<Y %d %d>\n", id,active)
ZZ(Z,id) // Delete output
        CHECK(Output::remove(id))
        REPLY("<O>\n")

ZZ(D,ACK,ON) // Enable PROG track diagnostics
        Diag::ACK = true;
ZZ(D,ACK,OFF) // Disable PROG track diagnostics
        Diag::ACK = false;
ZZ(D,CABS)  // Diagnostic display loco state table
        DCC::displayCabList(stream);
ZZ(D,RAM)  // Diagnostic display free RAM
        DIAG(F("Free memory=%d"), DCCTimer::getMinimumFreeMemory());
ZZ(D,CMD,ON) // Enable command input diagnostics
        Diag::CMD = true;
ZZ(D,CMD,OFF) // Disable command input diagnostics
        Diag::CMD = false;
ZZ(D,RAILCOM,ON) // Enable Railcom diagnostics
        Diag::RAILCOM = true;
ZZ(D,RAILCOM,OFF) // DIsable Railcom diagnostics
        Diag::RAILCOM = false;    
ZZ(D,WIFI,ON) // Enable Wifi diagnostics
        Diag::WIFI = true;
ZZ(D,WIFI,OFF) // Disable Wifi diagnostics
        Diag::WIFI = false; 
ZZ(D,ETHERNET,ON) // Enable Ethernet diagnostics
        Diag::ETHERNET = true;
ZZ(D,ETHERNET,OFF) // Disabel Ethernet diagnostics 
        Diag::ETHERNET = false;
ZZ(D,WIT,ON) // Enable Withrottle diagnostics
        Diag::WITHROTTLE = true;
ZZ(D,WIT,OFF) // Disable Withrottle diagnostics 
        Diag::WITHROTTLE = false;
ZZ(D,LCN,ON) // Enable LCN Diagnostics
        Diag::LCN = true;
ZZ(D,LCN,OFF) // Disabel LCN diagnostics
        Diag::LCN = false;
ZZ(D,WEBSOCKET,ON) // Enable Websocket diagnostics 
        Diag::WEBSOCKET = true;
ZZ(D,WEBSOCKET,OFF) // Disable wensocket diagnostics 
        Diag::WEBSOCKET = false;
            
#ifndef DISABLE_EEPROM  
ZZ(D,EEPROM,numentries) // Dump EEPROM contents
        EEStore::dump(numentries);
#endif


ZZ(D,ANOUT,vpin,position) // see <z vpin position>
        IODevice::writeAnalogue(vpin,position,0);
ZZ(D,ANOUT,vpin,position,profile) // see <z vpin position profile>
        IODevice::writeAnalogue(vpin,position,profile);
ZZ(D,SERVO,vpin,position) // Test servo
        IODevice::writeAnalogue(vpin,position,0);
ZZ(D,SERVO,vpin,position,profile) // Test servo
        IODevice::writeAnalogue(vpin,position,profile);
               
ZZ(D,ANIN,vpin) // Display analogue input value
        DIAG(F("VPIN=%u value=%d"), vpin, IODevice::readAnalogue(vpin));

ZZ(D,HAL,SHOW)  // Show HAL devices table
        IODevice::DumpAll();
ZZ(D,HAL,RESET) // Reset all HAL devices
        IODevice::reset();
ZZ(D,TT,vpin,steps) // Test turntable
        IODevice::writeAnalogue(vpin,steps,0);
ZZ(D,TT,vpin,steps,activity) // Test turntable
        IODevice::writeAnalogue(vpin,steps,activity);

ZZ(C,PROGBOOST) // Configute PROG track boost
        TrackManager::progTrackBoosted=true;
ZZ(C,RESET) // Reset and restart command station
        DCCTimer::reset();
ZZ(C,SPEED28) // Set all DCC speed commands as 28 step to old decoders
        DCC::setGlobalSpeedsteps(28); DIAG(F("28 Speedsteps"));
ZZ(C,SPEED128) // Set all DCC speed commands to 128 step (default)
        DCC::setGlobalSpeedsteps(128); DIAG(F("128 Speedsteps"));
ZZ(C,RAILCOM,ON) // Enable Railcom cutout 
        DIAG(F("Railcom %S"),DCCWaveform::setRailcom(true,false)?F("ON"):F("OFF"));
ZZ(C,RAILCOM,OFF) // Disable Railcom cutout
        DIAG(F("Railcom OFF")); DCCWaveform::setRailcom(false,false);
ZZ(C,RAILCOM,DEBUG) // Enable Railcom cutout for easy scope reading test
        DIAG(F("Railcom %S"), DCCWaveform::setRailcom(true,true)?F("ON"):F("OFF"));

#ifndef DISABLE_PROG
ZZ(D,ACK,LIMIT,value)  // Set ACK detection limit mA
        DCCACK::setAckLimit(value);                   LCD(1, F("Ack Limit=%dmA"), value); 
ZZ(D,ACK,MIN,value,MS) // Set ACK minimum duration mS
        DCCACK::setMinAckPulseDuration(value*1000L);  LCD(1, F("Ack Min=%dmS"), value); 
ZZ(D,ACK,MIN,value)    // Set ACK minimum duration uS
        DCCACK::setMinAckPulseDuration(value);        LCD(1, F("Ack Min=%duS"), value);  
ZZ(D,ACK,MAX,value,MS) // Set ACK maximum duration mS
        DCCACK::setMaxAckPulseDuration(value*1000L);  LCD(1, F("Ack Max=%dmS"), value);
ZZ(D,ACK,MAX,value)   // Set ACK maximum duration uS
        DCCACK::setMaxAckPulseDuration(value);        LCD(1, F("Ack Max=%duS"), value); 
ZZ(D,ACK,RETRY,value) // Set ACK retry count
        DCCACK::setAckRetry(value);                   LCD(1, F("Ack Retry=%d"), value);
#endif

#if defined(ARDUINO_ARCH_ESP32)
  // Dirty definition tricks because the executed check needs quote separation markers 
  // that should be invisible to the doc extractor.
  // The equivalent documentation will be extracted from the commented line below 
  // and the matchedFormat is hand modified to the correct format which includes quotes.

// (documented version) ZZ(C,WIFI,"ssid","password") // reconfigure stored wifi credentials 
ZZ_nodoc(C,WIFI,ssid,password) 
        CHECK(false, ssid and password must be in "quotes")
ZZ_nodoc(C,WIFI,marker1,ssid,marker2,password)
        DCCEXParser::matchedCommandFormat=F("C,WIFI,\"ssid\",\"password\""); // for error reporting
        CHECK(marker1==0x7777 && marker2==0x7777, ssid and password must be in "quotes")
        WifiESP::setup((const char*)(com + ssid), (const char*)(com + password), WIFI_HOSTNAME, IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif

ZZ(o,vpin) // Set neopixel on(vpin>0) or off(vpin<0)
        IODevice::write(abs(vpin),vpin>0);
ZZ(o,vpin,count)  // Set multiple neopixels on(vpin>0) or off(vpin<0)
        IODevice::writeRange(abs(vpin),vpin>0,count);
ZZ(o,vpin,r,g,b)  // Set neopixel colour
        CHECK(r>=0 && r<=0xff && g>=0 && g<=0xff && b>=0 && b<=0xff, r,g,b values range 0..255) 
        IODevice::writeAnalogueRange(abs(vpin),vpin>0,r<<8 | g,b,1);
ZZ(o,vpin,r,g,b,count) // Set multiple neopixels colour 
        CHECK(r>=0 && r<=0xff && g>=0 && g<=0xff && b>=0 && b<=0xff, r,g,b values range 0..255) 
        IODevice::writeAnalogueRange(abs(vpin),vpin>0,r<<8 | g,b,count);

ZZ(1)  // Power ON all tracks
        TrackManager::setTrackPower(TRACK_ALL, POWERMODE::ON);
ZZ(1,MAIN)  // Power on MAIN track
        TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::ON);
#ifndef DISABLE_PROG
ZZ(1,PROG)   // Power on PROG track
        TrackManager::setJoin(false); TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::ON);
ZZ(1,JOIN)  // JOIN prog track to MAIN and power
        TrackManager::setJoin(true); TrackManager::setTrackPower(TRACK_MODE_MAIN|TRACK_MODE_PROG, POWERMODE::ON);
#endif
ZZ(1,track) // Power on given track
        TrackManager::setTrackPower(POWERMODE::ON, (byte)track-'A');
ZZ(0)  // Power off all tracks
        TrackManager::setJoin(false); 
        TrackManager::setTrackPower(TRACK_ALL, POWERMODE::OFF);
ZZ(0,MAIN) // Power off MAIN track
        TrackManager::setJoin(false); 
        TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::OFF);
ZZ(0,PROG) // Power off PROG track
        TrackManager::setJoin(false); 
        TrackManager::progTrackBoosted=false;  
        // todo move to TrackManager Prog track boost mode will not outlive prog track off
        TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::OFF);
ZZ(0,track) // Power off given track
        TrackManager::setJoin(false);
	TrackManager::setTrackPower(POWERMODE::OFF, (byte)track-'a');

ZZ(c)   // Report main track currect (Deprecated)
        TrackManager::reportObsoleteCurrent(stream);

ZZ(a,address,subaddress,activate)  // Send DCC accessory command
        CHECK(activate==0 || activate ==1, invalid activate 0..1 )
        DCC::setAccessory(address, subaddress,activate ^ accessoryCommandReverse);
ZZ(a,address,subaddress,activate,onoff) // Send DCC accessory command with onoff control (TODO.. numbers) 
        CHECK(activate==0 || activate ==1, invalid activate 0..1 )
        CHECK(onoff>=0 && onoff<=2,invalid onoff 0..2 )
        DCC::setAccessory(address, subaddress,activate ^ accessoryCommandReverse ,onoff);
ZZ(a,linearaddress,activate) // send dcc accessory command      
        CHECK(activate==0 || activate ==1, invalid activate 0..1 )
        DCC::setAccessory((linearaddress - 1) / 4 + 1,(linearaddress - 1)  % 4 ,activate ^ accessoryCommandReverse);                                    
ZZ(A,address,value) // Send DCC extended accessory (Aspect) command
        DCC::setExtendedAccessory(address,value);

ZZ(w,loco,cv,value) // POM write cv on main track
        DCC::writeCVByteMain(loco,cv,value);
ZZ(r,loco,cv) // POM read cv on main track
        CHECK(DCCWaveform::isRailcom(),Railcom not active)
        EXPECT_CALLBACK
        DCC::readCVByteMain(loco,cv,callback_r);
ZZ(b,loco,cv,bit,bitvalue)  // POM write cv bit on main track
        DCC::writeCVBitMain(loco,cv,bit,bitvalue);
 
ZZ(m,LINEAR) // Set Momentum algorithm to linear acceleration
        DCC::linearAcceleration=true;
ZZ(m,POWER) // Set momentum algortithm to very based on difference between current speed and throttle seting
        DCC::linearAcceleration=false;
ZZ(m,loco,momentum)  // set momentum for loco (accel and braking)
        CHECK(DCC::setMomentum(loco,momentum,momentum))
ZZ(m,loco,accelerating,braking) // set momentum for loco
        CHECK(DCC::setMomentum(loco,accelerating,braking))

        // todo  reorder for more sensible doco. 
ZZ(W,cv,value,ignore1,ignore2) // (Deprecated) Write cv value on PROG track
        EXPECT_CALLBACK DCC::writeCVByte(cv,value, callback_W);        
ZZ(W,loco) // Write loco address on PROG track
        EXPECT_CALLBACK DCC::setLocoId(loco,callback_Wloco);
ZZ(W,CONSIST,loco,REVERSE) // Write consist address and reverse flag on PROG track 
        EXPECT_CALLBACK DCC::setConsistId(loco,true,callback_Wconsist);
ZZ(W,CONSIST,loco) // write consist address on PROG track       
        EXPECT_CALLBACK DCC::setConsistId(loco,false,callback_Wconsist);
ZZ(W,cv,value)   // Write cv value on PROG track
        EXPECT_CALLBACK DCC::writeCVByte(cv,value, callback_W);
ZZ(W,cv,bitvalue,bit) // Write cv bit on prog track
        EXPECT_CALLBACK DCC::writeCVBit(cv,bitvalue,bit,callback_W);
ZZ(V,cv,value) // Fast read cv with expected value
        EXPECT_CALLBACK DCC::verifyCVByte(cv,value, callback_Vbyte);
ZZ(V,cv,bit,bitvalue) // Fast read bit with expected value
        EXPECT_CALLBACK DCC::verifyCVBit(cv,bit,bitvalue,callback_Vbit);  
ZZ(B,cv,bit,bitvalue)  // Write cv bit
        EXPECT_CALLBACK DCC::writeCVBit(cv,bit,bitvalue,callback_B);
ZZ(R,cv,ignore1,ignore2) // (Deprecated) read cv value on PROG track
        EXPECT_CALLBACK DCC::readCV(cv,callback_R);
ZZ(R,cv) // Read cv
        EXPECT_CALLBACK DCC::verifyCVByte(cv, 0, callback_Vbyte);
ZZ(R)   // Read driveable loco id (may be long, short or consist)
        EXPECT_CALLBACK DCC::getLocoId(callback_Rloco);

#ifndef DISABLE_VDPY
ZZ_nodoc(@)   CommandDistributor::setVirtualLCDSerial(stream);
        REPLY( "<@ 0 0 \"DCC-EX v" VERSION "\">\n<@ 0 1 \"Lic GPLv3\">\n")
#endif
ZZ(@,display,row,string1) // Display string1 on line row (0 or 1) of LCD
  StringFormatter::lcd2(display,row,F("%s"),ZGETSTRING(string1));               
ZZ(-) // Clear loco state and reminder table
        DCC::forgetAllLocos();
ZZ(-,loco) // remove loco state amnd reminders
        DCC::forgetLoco(loco);      
ZZ(F,loco,DCCFREQ,freqvalue) // Set DC frequencey for loco   
        CHECK(freqvalue>=0 && freqvalue<=3) DCC::setDCFreq(loco,freqvalue);
ZZ(F,loco,function,onoff) // Set loco function ON/OFF
        CHECK(onoff==0 || onoff==1) DCC::setFn(loco,function,onoff);    
             
// ZZ(M,ignore,d0,d1,d2,d3,d4,d5) // Send up to 5 byte DCC packet on MAIN track (all d values in hex)
ZZ_nodoc(M,ignore,d0,d1,d2,d3,d4,d5) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4,(byte)d5}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(M,ignore,d0,d1,d2,d3,d4) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(M,ignore,d0,d1,d2,d3) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(M,ignore,d0,d1,d2) byte packet[]={(byte)d0,(byte)d1,(byte)d2}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(M,ignore,d0,d1) byte packet[]={(byte)d0,(byte)d1}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
// ZZ(P,ignore,d0,d1,d2,d3,d4,d5) // Send up to 5 byte DCC packet on PROG track (all d values in hex)
ZZ_nodoc(P,ignore,d0,d1,d2,d3,d4,d5) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4,(byte)d5}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(P,ignore,d0,d1,d2,d3,d4) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(P,ignore,d0,d1,d2,d3) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(P,ignore,d0,d1,d2) byte packet[]={(byte)d0,(byte)d1,(byte)d2}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ_nodoc(P,ignore,d0,d1) byte packet[]={(byte)d0,(byte)d1}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);

ZZ(J,O) // List turntable IDs
        REPLY("<jO")
        for (auto tto=Turntable::first(); tto; tto=tto->next()) if (!tto->isHidden()) REPLY(" %d",tto->getId())
        REPLY(">\n")
ZZ(J,O,id) // List turntable state
        auto tto=Turntable::get(id);
        if (!tto || tto->isHidden()) {REPLY("<jO %d X>\n", id) return true;}
        const FSH *todesc = nullptr;
#ifdef EXRAIL_ACTIVE
        todesc = RMFT2::getTurntableDescription(id);
#endif
        if (todesc == nullptr) todesc = F("");
        REPLY("<jO %d %d %d %d \"%S\">\n", id, tto->isEXTT(), tto->getPosition(), tto->getPositionCount(), todesc)
        
ZZ(J,P,id) // list turntable positions
        auto tto=Turntable::get(id);
        if (!tto || tto->isHidden()) {REPLY("<jP %d X>\n", id) return true;}      
        auto posCount = tto->getPositionCount();
        if (posCount==0) {REPLY("<jP X>\n") return true;}
        
        for (auto p = 0; p < posCount; p++) {
            const FSH *tpdesc = nullptr;
#ifdef EXRAIL_ACTIVE
            tpdesc = RMFT2::getTurntablePositionDescription(id, p);
#endif
            if (tpdesc == NULL) tpdesc = F("");
            REPLY("<jP %d %d %d \"%S\">\n", id, p, tto->getPositionAngle(p), tpdesc)
        }

// Track manager

ZZ(=) // list track manager states
        TrackManager::list(stream); 
ZZ(=,track,MAIN) // Set track to MAIN
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN))
ZZ(=,track,MAIN_INV) // Set track to MAIN inverted polatity
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN_INV))
ZZ(=,track,MAIN_AUTO) // Set track to MAIN with auto reversing
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN_AUTO))
ZZ(=,track,PROG) // Set track to PROG
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_PROG)) 
ZZ(=,track,OFF) // Set track power OFF
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_NONE))
ZZ(=,track,NONE) // Set track no output
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_NONE)) 
ZZ(=,track,EXT)  // Set track to use external sync
        CHECK(TrackManager::setTrackMode(track,TRACK_MODE_EXT))      

#ifdef BOOSTER_INPUT
ZZ_nodoc(=,track,BOOST)      CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST))
ZZ_nodoc(=,track,BOOST_INV)  CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST_INV))
ZZ_nodoc(=,track,BOOST_AUTO) CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST_AUTO))
#endif
ZZ(=,track,AUTO)  // Update track to auto reverse
        CHECK(TrackManager::orTrackMode(track, TRACK_MODIFIER_AUTO))
ZZ(=,track,INV) // Update track to inverse polarity
        CHECK(TrackManager::orTrackMode(track, TRACK_MODIFIER_INV))
ZZ(=,track,DC,loco) // Set track to DC
        CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC, loco))
ZZ(=,track,DC_INV,loco) // Set track to DC with inverted polarity
        CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC_INV, loco))
ZZ(=,track,DCX,loco) // Set track to DC with inverted polarity
        CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC_INV, loco))

ZZEND
