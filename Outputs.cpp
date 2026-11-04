/*
 *  © 2021 Neil McKechnie
 *  © 2021 Harald Barth
 *  © 2020-2021 Fred Decker
 *  © 2020-2021 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of Asbelos DCC API
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

#include "Outputs.h"
#include "StringFormatter.h"
#include "IODevice.h"

///////////////////////////////////////////////////////////////////////////////
// Static function to print all output states to stream in the form "<Y id state>"

void Output::printAll(Print *stream){
  for (Output *tt = Output::firstOutput; tt != NULL; tt = tt->nextOutput)
    StringFormatter::send(stream, F("<Y %d %d>\n"), tt->data.id, tt->data.active);
} // Output::printAll

///////////////////////////////////////////////////////////////////////////////
// Object method to activate / deactivate the Output state.

void  Output::activate(uint16_t s){
  s = (s>0);  // Make 0 or 1
  data.active = s;                     // if s>0, set status to active, else inactive
  // set state of output pin to HIGH or LOW depending on whether bit zero of iFlag is set to 0 (ACTIVE=HIGH) or 1 (ACTIVE=LOW)
  IODevice::write(data.pin, s ^ data.invert);  
}

///////////////////////////////////////////////////////////////////////////////
// Static function to locate Output object specified by ID 'n'.
//   Return NULL if not found.

Output* Output::get(uint16_t n){
  Output *tt;
  for(tt=firstOutput;tt!=NULL && tt->data.id!=n;tt=tt->nextOutput);
  return(tt);
}

///////////////////////////////////////////////////////////////////////////////
// Static function to delete Output object specified by ID 'n'.
//   Return false if not found.

bool Output::remove(uint16_t n){
  Output *tt,*pp=NULL;

  for(tt=firstOutput;tt!=NULL && tt->data.id!=n;pp=tt,tt=tt->nextOutput);

  if(tt==NULL) return false;
  
  if(tt==firstOutput)
    firstOutput=tt->nextOutput;
  else
    pp->nextOutput=tt->nextOutput;

  free(tt);

  return true;
  }

Output *Output::create(uint16_t id, VPIN pin, int iFlag, int v){
  Output *tt;

  if (pin > VPIN_MAX) return NULL;
  
  if(firstOutput==NULL){
    firstOutput=(Output *)calloc(1,sizeof(Output));
    tt=firstOutput;
  } else if((tt=get(id))==NULL){
    tt=firstOutput;
    while(tt->nextOutput!=NULL)
      tt=tt->nextOutput;
    tt->nextOutput=(Output *)calloc(1,sizeof(Output));
    tt=tt->nextOutput;
  }

  if(tt==NULL) return tt;
  
  tt->data.id=id;
  tt->data.pin=pin;
  tt->data.flags=iFlag;

  if(v==1){
    // sets status to 0 (INACTIVE) is bit 1 of iFlag=0, otherwise set to value of bit 2 of iFlag
    if (tt->data.setDefault) 
      tt->data.active = tt->data.defaultValue;
    else
      tt->data.active = 0;
  }
  IODevice::write(tt->data.pin, tt->data.active ^ tt->data.invert);

  return(tt);
}

///////////////////////////////////////////////////////////////////////////////

Output *Output::firstOutput=NULL;
