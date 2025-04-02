/*
 *  © 2023-2025, Barry Daniel  
 *  © 2025       Chris Harlow
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

//sensorCAM parser.cpp version 3.06  Jan 2025
#include "DCCEXParser.h"
#include "CamParser.h"
#include "FSH.h"

void camsend(byte camop,int16_t param1,int16_t param3) {
    DIAG(F("CamParser: %d %c %d %d"),CAMBaseVpin,camop,param1,param3);
    IODevice::writeAnalogue(CAMBaseVpin,param1,camop,param3);
}
  

// The CAMVPINS array will be filled by IO_EXSensorCam HAL drivers calling
// the CamParser::addVpin() function.
// The CAMBaseVpin is the one to be used when commands are given without a vpin. 
VPIN CamParser::CAMBaseVpin = 0; // no vpins yet known 
VPIN CamParser::CAMVPINS[] = {0,0,0,0};  // determines max # CAM's
int CamParser::vpcount=sizeof(CAMVPINS)/sizeof(CAMVPINS[0]);


  ZZ(N)                    // lists current base vpin and others available
    DIAG(F("Cam base vpin:%d"),CAMBaseVpin);
    for (auto i=0;i<vpcount;i++){
       if (CAMVPINS[i]==0) break;
       DIAG(F("EXSensorCam #%d vpin %d"),i+1,CAMVPINS[i]);
    }

  

  ZZ(N,V)                  // show version
    camsend('^',0,999);
  ZZ(N,F)                  //
    camsend(']',0,999);
  ZZ(N,Q)                  //
    camsend('Q',0,10);
  ZZ(N,camop)                  //
    CHECK(STRCHR_P((const char *)F("EGMRW"),camop))
    camsend(camop,0,999);

  ZZ(N,C,pin)              // change CAM base vpin or cam number from list
    CHECK(pin>=100 || (pin<vpcount && pin>=0))
    CAMBaseVpin=(pin>=100? pin:CAMVPINS[pin];
    DIAG(F("CAM base Vpin:%d "),pin,CAMBaseVpin);
 
  ZZ(N,camop,p1)           //send camop p1
    CHECK(STRCHR_P((const char *)F("ABFHILMNOPQRSTUV"),camop))
    camsend(camop,p1,999);
  
  ZZ(N,I,p1,p2)            //send camop p1 p2
    camsend('I',p1,p2);
  ZZ(N,J,p1,p2)            //send camop p1 p2
    camsend('J',p1,p2);
  ZZ(N,M,p1,p2)            //send camop p1 p2
    camsend('M',p1,p2);
  ZZ(N,N,p1,p2)            //send camop p1 p2
    camsend('N',p1,p2);
  ZZ(N,T,p1,p2)            //send camop p1 p2
    camsend('T',p1,p2);
  ZZ(N,vpin,rowY,colX)       //send 0x80 row col
    auto hold=CAMBaseVpin;
    CAMBaseVpin=vpin;
    camsend(0x80,rowY,colX);
    CAMBaseVpin=hold;

 ZZ(N,A,id,row,col)   
    CHECK(col<=316 && col>=0)
    CHECK(row<=236 && row>=0)
    CHECK(id<=97 && id >=0)
    auto hold=CAMBaseVpin;
    CAMBaseVpin=CAMBaseVpin + (id/10)*8 + id%10;   //translate from pseudo octal
    camsend(0x80,row,col)
    CAMBaseVpin=hold;

void CamParser::addVpin(VPIN pin) {
  // called by IO_EXSensorCam starting up a camera on a vpin
  byte slot=255;
  for (auto i=0;i<vpcount && slot==255;i++) {
    if (CAMVPINS[i]==0) {
      slot=i;
      CAMVPINS[slot]=pin;
    }
  }
  if (slot==255) {
    DIAG(F("No more than %d cameras supported"),vpcount);
    return;
  }
  if (slot==0) CAMBaseVpin=pin;
  DIAG(F("CamParser Registered cam #%dvpin %d"),slot+1,pin);
  // tell the DCCEXParser that we wish to filter commands
  DCCEXParser::setCamParserFilter(&parse);
}