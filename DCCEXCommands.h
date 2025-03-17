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
If the condition is false an error is genarated, resulting in an <X> reply.
The REPLY( format, ...) macro sends a formatted string to the stream. 
 
These macros are included into the DCCEXParser::execute function so
  strea, ringStream and other DCCEXParser variables are available in context. */

ZZBEGIN
ZZ(#)                    REPLY( "<# %d>\n", MAX_LOCOS)
ZZ(t,cab)                CHECK(cab>0) 
                        CommandDistributor::broadcastLoco(DCC::lookupSpeedTable(cab,false));
ZZ(t,cab,tspeed,direction)        CHECK(setThrottle(cab,tspeed,direction)) 
ZZ(t,ignore,cab,tspeed,direction) CHECK(setThrottle(cab,tspeed,direction)) 
// todo ZZ(f,cab,byte1)         CHECK(handleFunctionGroup(cab,byte1))
// todo ZZ(f,cab,byte1,byte2) CHECK(handleFunctionGroup(cab,byte1,byte2))

ZZ(T)                    Turnout::printAll(stream); // will <X> if none found
ZZ(T,id)                 CHECK(Turnout::remove(id)) 
ZZ(T,id,X)               auto tt=Turnout::get(id); CHECK(tt)  tt->print(stream);     
ZZ(T,id,T)               Turnout::setClosed(id, false);     
ZZ(T,id,C)               Turnout::setClosed(id, true);      
ZZ(T,id,value)           Turnout::setClosed(id, value==0);      
ZZ(T,id,SERVO,pin,low,high) CHECK(ServoTurnout::create(id, (VPIN)pin, (uint16_t)low, (uint16_t)high, 1)) 
ZZ(T,id,VPIN,pin)        CHECK(VpinTurnout::create(id, pin)) 
ZZ(T,id,DCC,addr,subadd) CHECK(DCCTurnout::create(id, addr, subadd)) 
ZZ(T,id,DCC,nn)          CHECK(DCCTurnout::create(id, (nn-1)/4+1, (nn-1)%4)) 
ZZ(T,id,addr,subadd)     CHECK(DCCTurnout::create(id, addr, subadd)) 
ZZ(T,id,pin,low,high)    CHECK(ServoTurnout::create(id, (VPIN)pin,low,high,1)) 
ZZ(S,id,pin,pullup)      CHECK(Sensor::create(id,pin,pullup)) 
ZZ(S,id)                 CHECK(Sensor::remove(id))
ZZ(S)                    for (auto *tt = Sensor::firstSensor; tt; tt = tt->nextSensor) {
                            REPLY("<Q %d %d %d>\n", tt->data.snum, tt->data.pin, tt->data.pullUp)
                         }           
ZZ(J,M)                  Stash::list(stream);
ZZ(J,M,stash_id)         Stash::list(stream, stash_id);
ZZ(J,M,CLEAR,ALL)        Stash::clearAll(); 
ZZ(J,M,CLEAR,stash_id)   Stash::clear(stash_id); 
ZZ(J,M,stashId,locoId)   Stash::set(stashId,locoId); 
ZZ(J,M,CLEAR,ANY,locoId) Stash::clearAny(locoId);
ZZ(J,C)                  REPLY("<jC %d>\n", CommandDistributor::retClockTime())
ZZ(J,C,mmmm,nn)         CommandDistributor::setClockTime(mmmm, nn, 1);
            
ZZ(J,G) TrackManager::reportGauges(stream);   // <g limit...limit>     
ZZ(J,I) TrackManager::reportCurrent(stream);   // <g limit...limit>     
ZZ(J,L,display,row) TrackManager::reportCurrentLCD(display,row);   // Track power status     
ZZ(J,A) REPLY( "<jA>\n") // <JA> intercepted by EXRAIL// <JA> returns automations/routes
#ifdef EXRAIL_ACTIVE
ZZ(J,R) REPLY("<jR")
        SENDFLASHLIST(stream,RMFT2::rosterIdList)
        REPLY(">\n");      
ZZ(J,R,id)      auto rosterName= RMFT2::getRosterName(id);
                if (!rosterName) rosterName=F("");
                auto functionNames= RMFT2::getRosterFunctions(id);
                if (!functionNames) functionNames=RMFT2::getRosterFunctions(0);
                if (!functionNames) functionNames=F("");
                REPLY("<jR %d \"%S\" \"%S\">\n",id, rosterName, functionNames)
#else
ZZ(J,R) REPLY("<jR>\n") // <JR> returns empty roster list
#endif                
ZZ(J,T)    // <JT> returns turnout list 
            REPLY("<jT")
            for ( auto t=Turnout::first(); t; t=t->next()) if (!t->isHidden())  REPLY(" %d",t->getId()) 
            REPLY(">\n");
ZZ(J,T,id)  auto t=Turnout::get(id);
            if (!t || t->isHidden()) REPLY("<jT %d X>\n",id)
            else {
                const FSH *tdesc=nullptr;
#ifdef EXRAIL_ACTIVE
                tdesc = RMFT2::getTurnoutDescription(id);
#endif
                if (!tdesc) tdesc = F("");
                REPLY("<jT %d %c \"%S\">\n",id,t->isThrown()?'T':'C',tdesc)
            }
ZZ(z,vpin)   // <z vpin | -vpin> 
            if (vpin>0) IODevice::write(vpin,HIGH);
            else IODevice::write(-vpin,LOW);
ZZ(z,vpin,analog,profile,duration) IODevice::writeAnalogue(vpin,analog,profile,duration);
ZZ(z,vpin,analog,profile) IODevice::writeAnalogue(vpin,analog,profile,0);
ZZ(z,vpin,analog) IODevice::writeAnalogue(vpin,analog,0,0);
     
// ==========================
// Turntable - no support if no HAL
// <I> - list all
// <I id> - broadcast type and current position
// <I id DCC> - create DCC - This is TBA
// <I id steps> - operate (DCC)
// <I id steps activity> - operate (EXTT)
// <I id ADD position value> - add position
// <I id EXTT i2caddress vpin home> - create EXTT

ZZ(I)     return Turntable::printAll(stream);

ZZ(I,id)  // <I id> broadcast type and current position    
         auto tto = Turntable::get(id);
         CHECK(tto)
         REPLY("<I %d %d>\n", tto->isEXTT(), tto->getPosition())
        

ZZ(I,id,position) // <I id position> - rotate a DCC turntable
         auto tto = Turntable::get(id);
        CHECK(tto)         
        CHECK(!tto->isEXTT())
        CHECK(tto->setPosition(id,position))

ZZ(I,id,DCC,home) 
        auto tto = Turntable::get(id);
        CHECK(tto)
        CHECK(home >=0 && home <= 3600)
        CHECK(DCCTurntable::create(id)) 
        tto = Turntable::get(id);
        CHECK(tto)
        tto->addPosition(0, 0, home);
        REPLY("<I>\n")

ZZ(I,id,position,activity)
        auto tto = Turntable::get(id); 
        CHECK(tto)
        CHECK(tto->isEXTT())
        CHECK(tto->setPosition(id, position,activity))
    
ZZ(I,id,EXTT,vpin,home) // <I id EXTT vpin home> create an EXTT turntable
        auto tto = Turntable::get(id);
        CHECK(!tto && home >= 0 && home <= 3600)
        CHECK(EXTTTurntable::create(id, (VPIN)vpin))
        tto = Turntable::get(id);
        tto->addPosition(0, 0, home);
        REPLY("<I>\n")


ZZ(I,id,ADD,position,value,angle) // <I id ADD position value angle> add a position
     auto tto = Turntable::get(id);
     // tto must exist, no more than 48 positions, angle 0 - 3600
     CHECK(tto && position <= 48 && angle >=0  && angle <= 3600)
     tto->addPosition(id,value,angle);
     REPLY("<I>\n")

 ZZ(Q)  Sensor::printAll(stream);

 ZZ(s) // STATUS <s>
        REPLY("<iDCC-EX V-%S / %S / %S G-%S>\n", F(VERSION), F(ARDUINO_TYPE), DCC::getMotorShieldName(), F(GITHUB_SHA))
        CommandDistributor::broadcastPower(); // <s> is the only "get power status" command we have
        Turnout::printAll(stream); //send all Turnout states
        Sensor::printAll(stream);  //send all Sensor  states
               

#ifndef DISABLE_EEPROM
    ZZ(E) // STORE EPROM <E>
        EEStore::store();
        REPLY("<e %d %d %d>\n", EEStore::eeStore->data.nTurnouts, EEStore::eeStore->data.nSensors, EEStore::eeStore->data.nOutputs)

    ZZ(e) // CLEAR EPROM <e>
        EEStore::clear();
        REPLY("<O>\n")

#endif

ZZ(Z,id,active) auto o = Output::get(id);
                CHECK(o)
                o->activate(active);
                REPLY("<Y %d %d>\n", id,active)

ZZ(Z,id,pin,iflag) // <Z ID PIN IFLAG>
                CHECK(id > 0 && iflag >= 0 && iflag <= 7 )
                CHECK(Output::create(id,pin,iflag, 1))
                REPLY("<O>\n")
ZZ(Z,id)        CHECK(Output::remove(id))
                REPLY("<O>\n")

ZZ(Z)           // <Z> list Output definitions
                bool gotone = false;
                for (auto *tt = Output::firstOutput; tt ; tt = tt->nextOutput){
                    gotone = true;
                    REPLY("<Y %d %d %d %d>\n", 
                    tt->data.id, tt->data.pin, tt->data.flags, tt->data.active)    
                }
                CHECK(gotone)
ZZ(D,ACK,ON) Diag::ACK = true;
ZZ(D,ACK,OFF) Diag::ACK = false;
ZZ(D,CABS)    DCC::displayCabList(stream);
ZZ(D,RAM)    DIAG(F("Free memory=%d"), DCCTimer::getMinimumFreeMemory());
ZZ(D,CMD,ON) Diag::CMD = true;
ZZ(D,CMD,OFF) Diag::CMD = false;
ZZ(D,RAILCOM,ON) Diag::RAILCOM = true;
ZZ(D,RAILCOM,OFF) Diag::RAILCOM = false;    
ZZ(D,WIFI,ON) Diag::WIFI = true;
ZZ(D,WIFI,OFF) Diag::WIFI = false; 
ZZ(D,ETHERNET,ON) Diag::ETHERNET = true;
ZZ(D,ETHERNET,OFF) Diag::ETHERNET = false;
ZZ(D,WIT,ON) Diag::WITHROTTLE = true;
ZZ(D,WIT,OFF) Diag::WITHROTTLE = false;
ZZ(D,LCN,ON) Diag::LCN = true;
ZZ(D,LCN,OFF) Diag::LCN = false;
ZZ(D,WEBSOCKET,ON) Diag::WEBSOCKET = true;
ZZ(D,WEBSOCKET,OFF) Diag::WEBSOCKET = false;
            
#ifndef DISABLE_EEPROM  
ZZ(D,EEPROM,numentries) EEStore::dump(numentries);
#endif


ZZ(D,ANOUT,vpin,position) IODevice::writeAnalogue(vpin,position,0);
ZZ(D,ANOUT,vpin,position,profile) IODevice::writeAnalogue(vpin,position,profile);
ZZ(D,SERVO,vpin,position) IODevice::writeAnalogue(vpin,position,0);
ZZ(D,SERVO,vpin,position,profile) IODevice::writeAnalogue(vpin,position,profile);
               
ZZ(D,ANIN,vpin)// <D ANIN vpin>  Display analogue input value
        DIAG(F("VPIN=%u value=%d"), vpin, IODevice::readAnalogue(vpin));

ZZ(D,HAL,SHOW)          IODevice::DumpAll();
ZZ(D,HAL,RESET)         IODevice::reset();
ZZ(D,TT,vpin,steps)          IODevice::writeAnalogue(vpin,steps,0);
ZZ(D,TT,vpin,steps,activity) IODevice::writeAnalogue(vpin,steps,activity);

ZZ(C,PROGBOOST) TrackManager::progTrackBoosted=true;
ZZ(C,RESET)        DCCTimer::reset();
ZZ(C,SPEED28) DCC::setGlobalSpeedsteps(28); DIAG(F("28 Speedsteps"));
ZZ(C,SPEED128) DCC::setGlobalSpeedsteps(128); DIAG(F("128 Speedsteps"));
ZZ(C,RAILCOM,ON) DIAG(F("Railcom %S"),DCCWaveform::setRailcom(true,false)?F("ON"):F("OFF"));
ZZ(C,RAILCOM,OFF) DIAG(F("Railcom OFF")); DCCWaveform::setRailcom(false,false);
ZZ(C,RAILCOM,DEBUG) DIAG(F("Railcom %S"), DCCWaveform::setRailcom(true,true)?F("ON"):F("OFF"));

#ifndef DISABLE_PROG
ZZ(D,ACK,LIMIT,value)    DCCACK::setAckLimit(value);                   LCD(1, F("Ack Limit=%dmA"), value); 
ZZ(D,ACK,MIN,value,MS)   DCCACK::setMinAckPulseDuration(value*1000L);  LCD(1, F("Ack Min=%dmS"), value); 
ZZ(D,ACK,MIN,value)      DCCACK::setMinAckPulseDuration(value);        LCD(1, F("Ack Min=%duS"), value);  
ZZ(D,ACK,MAX,value,MS)   DCCACK::setMaxAckPulseDuration(value*1000L);  LCD(1, F("Ack Max=%dmS"), value);
ZZ(D,ACK,MAX,value)      DCCACK::setMaxAckPulseDuration(value);        LCD(1, F("Ack Max=%duS"), value); 
ZZ(D,ACK,RETRY,value)    DCCACK::setAckRetry(value);                   LCD(1, F("Ack Retry=%d"), value);
#endif
#if defined(ARDUINO_ARCH_ESP32)
// currently this only works on ESP32
ZZ(C,WIFI,marker1,ssid,marker2,password)
 	// <C WIFI SSID PASSWORD>
    CHECK(marker1==0x7777 && marker2==0x7777)
    WifiESP::setup((const char*)(com + ssid), (const char*)(com + password), WIFI_HOSTNAME, IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif

ZZ(o,vpin)              IODevice::write(abs(vpin),vpin>0);
ZZ(o,vpin,count)        IODevice::writeRange(abs(vpin),vpin>0,count);
ZZ(o,vpin,r,g,b)        CHECK(r>-0 && r<=0xff) CHECK(g>-0 && g<=0xff) CHECK(b>-0 && b<=0xff) 
                        IODevice::writeAnalogueRange(abs(vpin),vpin>0,r<<8 | g,b,1);
ZZ(o,vpin,r,g,b,count)  CHECK(r>-0 && r<=0xff) CHECK(g>-0 && g<=0xff) CHECK(b>-0 && b<=0xff) 
                        IODevice::writeAnalogueRange(abs(vpin),vpin>0,r<<8 | g,b,count);

ZZ(1)                   TrackManager::setTrackPower(TRACK_ALL, POWERMODE::ON);
ZZ(1,MAIN)              TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::ON);
#ifndef DISABLE_PROG
ZZ(1,PROG)              TrackManager::setJoin(false); TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::ON);
ZZ(1,JOIN)              TrackManager::setJoin(true); TrackManager::setTrackPower(TRACK_MODE_MAIN|TRACK_MODE_PROG, POWERMODE::ON);
#endif
ZZ(1,letter) CHECK(letter>='A' && letter<='H')   TrackManager::setTrackPower(POWERMODE::ON, (byte)letter-'A');

ZZ(0)   TrackManager::setJoin(false); TrackManager::setTrackPower(TRACK_ALL, POWERMODE::OFF);
ZZ(0,MAIN)TrackManager::setJoin(false); TrackManager::setTrackPower(TRACK_MODE_MAIN, POWERMODE::OFF);
ZZ(0,PROG) TrackManager::setJoin(false); TrackManager::progTrackBoosted=false;  
           // todo move to TrackManager Prog track boost mode will not outlive prog track off
	       TrackManager::setTrackPower(TRACK_MODE_PROG, POWERMODE::OFF);
ZZ(0,letter) CHECK(letter>='A' && letter <='H') 
          TrackManager::setJoin(false);
	      TrackManager::setTrackPower(POWERMODE::OFF, (byte)letter-'a');

ZZ(!)    DCC::estopAll(); // this broadcasts speed 1(estop) and sets all reminders to speed 1.
ZZ(c)    TrackManager::reportObsoleteCurrent(stream);

ZZ(a,address,subaddress,activate)   DCC::setAccessory(address, subaddress,activate ^ accessoryCommandReverse);
ZZ(a,address,subaddress,activate,onoff) CHECK(onoff>=0 && onoff<-2)
                                    DCC::setAccessory(address, subaddress,activate ^ accessoryCommandReverse ,onoff);
ZZ(a,linearaddress,activate)      
    DCC::setAccessory((linearaddress - 1) / 4 + 1,(linearaddress - 1)  % 4 ,activate ^ accessoryCommandReverse);                                    
ZZ(A,address,value)                 DCC::setExtendedAccessory(address,value);

ZZ(w,cab,cv,value)   DCC::writeCVByteMain(cab,cv,value);
ZZ(r,cab,cv) 
      CHECK(DCCWaveform::isRailcom())
      EXPECT_CALLBACK
      DCC::readCVByteMain(cab,cv,callback_r);
 ZZ(b,cab,cv,bit,value)     DCC::writeCVBitMain(cab,cv,bit,value);
 ZZ(m,LINEAR) DCC::linearAcceleration=true;
 ZZ(m,POWER)  DCC::linearAcceleration=false;
 ZZ(m,cab,momentum)  CHECK(DCC::setMomentum(cab,momentum,momentum))
 ZZ(m,cab,momentum,braking)  CHECK(DCC::setMomentum(cab,momentum,braking))

 ZZ(W,cv,value,ignore1,ignore2) EXPECT_CALLBACK DCC::writeCVByte(cv,value, callback_W4);
ZZ(W,cab)  EXPECT_CALLBACK DCC::setLocoId(cab,callback_Wloco);
ZZ(W,CONSIST,cab,REVERSE) EXPECT_CALLBACK DCC::setConsistId(cab,true,callback_Wconsist);
ZZ(W,CONSIST,cab)        EXPECT_CALLBACK DCC::setConsistId(cab,false,callback_Wconsist);
ZZ(W,cv,value)        EXPECT_CALLBACK DCC::writeCVByte(cv,value, callback_W);
ZZ(W,cv,value,bit)    EXPECT_CALLBACK DCC::writeCVBit(cv,value,bit,callback_W);
ZZ(V,cv,value)        EXPECT_CALLBACK DCC::verifyCVByte(cv,value, callback_Vbyte);
ZZ(V,cv,bit,value)    EXPECT_CALLBACK DCC::verifyCVBit(cv,bit,value,callback_Vbit);  

ZZ(B,cv,bit,value)    EXPECT_CALLBACK DCC::writeCVBit(cv,bit,value,callback_B);
ZZ(R,cv,ignore1,ignore2) EXPECT_CALLBACK DCC::readCV(cv,callback_R);
ZZ(R,cv)               EXPECT_CALLBACK DCC::verifyCVByte(cv, 0, callback_Vbyte);
ZZ(R)              EXPECT_CALLBACK DCC::getLocoId(callback_Rloco);

#ifndef DISABLE_VDPY
ZZ(@)   CommandDistributor::setVirtualLCDSerial(stream);
        REPLY( "<@ 0 0 \"DCC-EX v" VERSION "\">\n<@ 0 1 \"Lic GPLv3\">\n")
#endif 
               
ZZ(-) DCC::forgetAllLocos();
ZZ(-,cab) DCC::forgetLoco(cab);      
ZZ(F,cab,DCCFREQ,value) CHECK(value>=0 && value<=3) DCC::setDCFreq(cab,value);
ZZ(F,cab,function,value) CHECK(value==0 || value==1) DCC::setFn(cab,function,value);    
             

ZZ(M,ignore,d0,d1,d2,d3,d4,d5) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4,(byte)d5}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(M,ignore,d0,d1,d2,d3,d4) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(M,ignore,d0,d1,d2,d3) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(M,ignore,d0,d1,d2) byte packet[]={(byte)d0,(byte)d1,(byte)d2}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(M,ignore,d0,d1) byte packet[]={(byte)d0,(byte)d1}; DCCWaveform::mainTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(P,ignore,d0,d1,d2,d3,d4,d5) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4,(byte)d5}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(P,ignore,d0,d1,d2,d3,d4) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3,(byte)d4}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(P,ignore,d0,d1,d2,d3) byte packet[]={(byte)d0,(byte)d1,(byte)d2,(byte)d3}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(P,ignore,d0,d1,d2) byte packet[]={(byte)d0,(byte)d1,(byte)d2}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);
ZZ(P,ignore,d0,d1) byte packet[]={(byte)d0,(byte)d1}; DCCWaveform::progTrack.schedulePacket(packet,sizeof(packet),3);

ZZ(J,O) REPLY("<jO")
        for (auto tto=Turntable::first(); tto; tto=tto->next()) if (!tto->isHidden()) REPLY(" %d",tto->getId())
        REPLY(">\n")
ZZ(J,O,id) auto tto=Turntable::get(id);
        if (!tto || tto->isHidden()) {REPLY("<jO %d X>\n", id) return true;}
        const FSH *todesc = nullptr;
#ifdef EXRAIL_ACTIVE
        todesc = RMFT2::getTurntableDescription(id);
#endif
        if (todesc == nullptr) todesc = F("");
        REPLY("<jO %d %d %d %d \"%S\">\n", id, tto->isEXTT(), tto->getPosition(), tto->getPositionCount(), todesc)
        
ZZ(J,P,id) auto tto=Turntable::get(id);
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
ZZ(=) TrackManager::list(stream); 
ZZ(=,track,MAIN)      CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN))
ZZ(=,track,MAIN_INV)  CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN_INV))
ZZ(=,track,MAIN_AUTO) CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_MAIN_AUTO))
ZZ(=,track,PROG)      CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_PROG)) 
ZZ(=,track,OFF)       CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_NONE))
ZZ(=,track,NONE)      CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_NONE)) 
ZZ(=,track,EXT)       CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_EXT))      

#ifdef BOOSTER_INPUT
ZZ(=,track,BOOST)      CHECK(TRACK_MODE_BOOST)      CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST))
ZZ(=,track,BOOST_INV)  CHECK(TRACK_MODE_BOOST_INV)  CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST_INV))
ZZ(=,track,BOOST_AUTO) CHECK(TRACK_MODE_BOOST_AUTO) CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track,TRACK_MODE_BOOST_AUTO))
#endif
ZZ(=,track,AUTO)          CHECK(track>='A' && track<='H') CHECK(TrackManager::orTrackMode(track, TRACK_MODIFIER_AUTO))
ZZ(=,track,INV)           CHECK(track>='A' && track<='H') CHECK(TrackManager::orTrackMode(track, TRACK_MODIFIER_INV))
ZZ(=,track,DC,locoid)     CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC, locoid))
ZZ(=,track,DC_INV,locoid) CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC_INV, locoid))
ZZ(=,track,DCX,locoid)    CHECK(track>='A' && track<='H') CHECK(TrackManager::setTrackMode(track, TRACK_MODE_DC_INV, locoid))

ZZEND
