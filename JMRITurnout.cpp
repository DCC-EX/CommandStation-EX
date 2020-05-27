/**********************************************************************

Accessories.cpp
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/
/**********************************************************************

DCC++ BASE STATION can keep track of the direction of any turnout that is controlled
by a DCC stationary accessory decoder.  All turnouts, as well as any other DCC accessories
connected in this fashion, can always be operated using the DCC BASE STATION Accessory command:

  <a ADDRESS SUBADDRESS ACTIVATE>

However, this general command simply sends the appropriate DCC instruction packet to the main tracks
to operate connected accessories.  It does not store or retain any information regarding the current
status of that accessory.

To have this sketch store and retain the direction of DCC-connected turnouts, as well as automatically
invoke the required <a> command as needed, first define/edit/delete such turnouts using the following
variations of the "T" command:

  <T ID ADDRESS SUBADDRESS>:   creates a new turnout ID, with specified ADDRESS and SUBADDRESS
                               if turnout ID already exists, it is updated with specificed ADDRESS and SUBADDRESS
                               returns: <O> if successful and <X> if unsuccessful (e.g. out of memory)

  <T ID>:                      deletes definition of turnout ID
                               returns: <O> if successful and <X> if unsuccessful (e.g. ID does not exist)

  <T>:                         lists all defined turnouts
                               returns: <H ID ADDRESS SUBADDRESS THROW> for each defined turnout or <X> if no turnouts defined

where

  ID: the numeric ID (0-32767) of the turnout to control
  ADDRESS:  the primary address of the decoder controlling this turnout (0-511)
  SUBADDRESS: the subaddress of the decoder controlling this turnout (0-3)

Once all turnouts have been properly defined, use the <E> command to store their definitions to EEPROM.
If you later make edits/additions/deletions to the turnout definitions, you must invoke the <E> command if you want those
new definitions updated in the EEPROM.  You can also clear everything stored in the EEPROM by invoking the <e> command.

To "throw" turnouts that have been defined use:

  <T ID THROW>:                sets turnout ID to either the "thrown" or "unthrown" position
                               returns: <H ID THROW>, or <X> if turnout ID does not exist

where

  ID: the numeric ID (0-32767) of the turnout to control
  THROW: 0 (unthrown) or 1 (thrown)

When controlled as such, the Arduino updates and stores the direction of each Turnout in EEPROM so
that it is retained even without power.  A list of the current directions of each Turnout in the form <H ID THROW> is generated
by this sketch whenever the <s> status command is invoked.  This provides an efficient way of initializing
the directions of any Turnouts being monitored or controlled by a separate interface or GUI program.

**********************************************************************/

//#include "EEStore.h"
//#include <EEPROM.h>
#include "JMRITurnout.h"
#include "StringParser.h"
#include "DCC.h"

void JMRITurnout::parse(Stream & stream, int params, int p[]) {
  
  switch(params){
        case 0:                    // <T>
            showAll(stream);                  // verbose show
            return;
        case 1:                     // <T id>
            if (!remove(p[0])) break;
            StringParser::send(stream,F("<O>"));
            return;
        case 2:                     // <T id 0|1>
        if (!activate(p[0],p[1])) break;
            show(stream,p[0],false);
            return;

        case 3:                     // <T id addr subaddr>
            if (!create(p[0],p[1],p[2])) break;
            StringParser::send(stream,F("<O>"));
            break;
        default:
            break; // will <x>                    
        }
        StringParser::send(stream,F("<X>"));
}

///////////////// ALL PRIVATE BELOW HERE //////////////////
 JMRITurnout::TurnoutEntry JMRITurnout::table[MAX_TURNOUTS];

 bool JMRITurnout::create(int id, int add, byte subAdd){
  if (id<0 || id>=MAX_TURNOUTS || table[id].address!=0) return false;
  table[id].address=add;
  table[id].subAddress=subAdd;
  table[id].thrown=false;
  return true;
}


bool JMRITurnout::invalid(int id) {
     return  id<0 || id>=MAX_TURNOUTS || table[id].address==0;
}
   
bool JMRITurnout::activate(int id,bool thrown){
   if (invalid(id)) return false;
   table[id].thrown=thrown;
   DCC::setAccessory(table[id].address,table[id].subAddress,table[id].thrown);  
   return true;
}

bool JMRITurnout::remove(int id){
   if (invalid(id)) return false;
   table[id].address=0;
   return true;
}


bool JMRITurnout::show(Stream & stream ,int id, bool all){
  if (invalid(id)) return false;
  if (all) StringParser::send(stream, F("<H %d %d %d %d>"), 
                              id, table[id].address, 
                              table[id].subAddress, table[id].thrown);
   else StringParser::send(stream, F("<H %d %d>"), 
                              id, table[id].thrown);
   return true;                           
}

void JMRITurnout::showAll(Stream & stream ){
  for (int id=0;id<MAX_TURNOUTS;id++) {
    if (table[id].address!=0) show(stream,id,true);    
  }
}


///////////////////////////////////////////////////////////////////////////////
#ifdef NOT_YET_IMPLEMENTED
void Turnout::load(){
  struct TurnoutData data;
  Turnout *tt;

  for(int i=0;i<EEStore::eeStore->data.nTurnouts;i++){
    EEPROM.get(EEStore::pointer(),data);
    tt=create(data.id,data.address,data.subAddress);
    tt->data.tStatus=data.tStatus;
    tt->num=EEStore::pointer();
    EEStore::advance(sizeof(tt->data));
  }
}

///////////////////////////////////////////////////////////////////////////////

void Turnout::store(){
  Turnout *tt;

  tt=firstTurnout;
  EEStore::eeStore->data.nTurnouts=0;

  while(tt!=NULL){
    tt->num=EEStore::pointer();
    EEPROM.put(EEStore::pointer(),tt->data);
    EEStore::advance(sizeof(tt->data));
    tt=tt->nextTurnout;
    EEStore::eeStore->data.nTurnouts++;
  }

}
#endif
///////////////////////////////////////////////////////////////////////////////
