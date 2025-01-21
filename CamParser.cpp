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

const int16_t ver=30177;
const int16_t ve =2899;


// The CAMVPINS array will be filled by IO_EXSensorCam HAL drivers calling
// the CamParser::addVpin() function.
// The CAMBaseVpin is the one to be used when commands are given without a vpin. 
VPIN CamParser::CAMBaseVpin = 0; // no vpins yet known 
VPIN CamParser::CAMVPINS[] = {0,0,0,0};  // determines max # CAM's
int CamParser::vpcount=sizeof(CAMVPINS)/sizeof(CAMVPINS[0]);

void CamParser::parse(Print * stream, byte & opcode, byte & paramCount, int16_t p[]) {
  if (opcode!='N') return; // this is not for us. 
  if (parseN(stream,paramCount,p)) opcode=0; // we have consumed this
  // If we fail, the caller will <X> the <N command. 
}
     
bool CamParser::parseN(Print * stream, byte paramCount, int16_t p[]) {
  (void)stream;  // probably unused parameter   
  if (CAMBaseVpin==0) CAMBaseVpin=CAMVPINS[0];  // default to CAM 1.
  VPIN vpin=CAMBaseVpin;   //use current CAM selection

  if (paramCount==0) { 
    DIAG(F("Cam base vpin:%d"),CAMBaseVpin);
    for (auto i=0;i<vpcount;i++){
       if (CAMVPINS[i]==0) break;
       DIAG(F("EXSensorCam #%d vpin %d"),i+1,CAMVPINS[i]);
    }
    return true; 
  }
  uint8_t camop=p[0]; // cam oprerator 
  int param1=0;
  int16_t param3=9999;   // =0 could invoke parameter changes. & -1 gives later errors

  if(camop=='C'){ 
    if(p[1]>=100) CAMBaseVpin=p[1];
    if(p[1]<=vpcount && p[1]>0)    CAMBaseVpin=CAMVPINS[p[1]-1];
    DIAG(F("CAM base Vpin: %c %d "),p[0],CAMBaseVpin);
    return true;
  }
  if (camop<100) {               //switch CAM# if p[1] dictates
    if(p[1]>=100 && p[1]<=(vpcount*100+99)) {  //limits to CAM# 1 to 4 for now
      vpin=CAMVPINS[p[1]/100-1];
      CAMBaseVpin=vpin;     
      DIAG(F("switching to CAM %d baseVpin:%d"),p[1]/100,vpin); 
      p[1]=p[1]%100;             //strip off CAM #
    } 
  }
  if (CAMBaseVpin==0) {DIAG(F("<n Error: Invalid CAM selected, default to CAM1>"));
    return false; // cam not defined
  }	
 
      // send UPPER case to sensorCAM to flag binary data from a DCCEX-CS parser  
  switch(paramCount) {    
    case 1:                          //<N ver> produces '^'
      if((camop == 'V') || (p[0] == ve) || (p[0] == ver) ) camop='^'; 
      if (STRCHR_P((const char *)F("EFGMQRVW^"),camop) == nullptr) return false;
      if (camop=='Q') param3=10;     //<NQ> for activation state of all 10 banks of sensors
      if (camop=='F') camop=']';     //<NF> for Reset/Finish webCAM.
      break;    // F Coded as ']' else conflicts with <Nf %%>
    
    case 2:                          //<N camop p1>  
      if (STRCHR_P((const char *)F("ABFHILMNOPQRSTUV"),camop)==nullptr) return false;
      param1=p[1];
      break;
    
    case 3:              //<N vpin rowY colx > or <N cmd p1 p2>
      if (p[0]>=100) {   //vpin - i.e. NOT 'A' through 'Z'
        if (p[1]>236 || p[1]<0) return false;     //row
        if (p[2]>316 || p[2]<0) return false;     //column
        camop=0x80;      // special 'a' case for IO_SensorCAM
        vpin = p[0];
      }else if (STRCHR_P((const char *)F("IJMNT"),camop) == nullptr) return false; 
	  camop=p[0];
      param1 = p[1];  
      param3 = p[2];
      break;
    
    case 4:          //<N a id row col> 
      if (camop!='A') return false;          //must start with 'a' 
      if (p[3]>316 || p[3]<0) return false;
      if (p[2]>236 || p[2]<0) return false;
      if (p[1]>97 || p[1]<0) return false;   //treat as bsNo.
      vpin = vpin + (p[1]/10)*8 + p[1]%10;   //translate p[1]
      camop=0x80;    // special 'a' case for IO_SensorCAM
      param1=p[2];   // row
      param3=p[3];   // col
      break;

    default:
      return false;
  }
  DIAG(F("CamParser: %d %c %d %d"),vpin,camop,param1,param3);
  IODevice::writeAnalogue(vpin,param1,camop,param3);
  return true;
}

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