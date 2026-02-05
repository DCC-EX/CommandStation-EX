#include "DCCConsist.h"
#include "LocoSlot.h"
#include "StringFormatter.h"
#include "DIAG.h"

bool DCCConsist::parse(Print * stream, byte params, int16_t p[]) {
    if (params==0) {
          for (auto slot=LocoSlot::getFirst();slot;slot=slot->getNext()) {
            if (slot->isConsistLead()) {
              StringFormatter::send(stream,F("<^"));
              for (auto cslot=slot;cslot;cslot=cslot->getConsistNext()) {
                StringFormatter::send(stream,
                    cslot->isConsistReverse() ? F(" -%d") : F(" %d"),
                    cslot->getLoco());
              }                 
              StringFormatter::send(stream,F(" >\n"));
            }
          }
        return true;
    }
    // Detect any invalid locoids or duplicates 
    for (byte i=0;i<params;i++) {
        uint16_t locoId=abs(p[i]);
        if (locoId<1 || locoId>10239) {
            StringFormatter::send(stream,F("<* Invalid locoid %d *>"),p[i]);
            return false;
        }
        for (byte j=i+1;j<params;j++) {
            if (abs(p[j])==locoId) {
                StringFormatter::send(stream,F("<* Duplicate locoid %d *>"),p[i]);
                return false;
            }
        }
    }
    // Cross check other consists  
    // if p[0] is a consist lead, kill its consist first
    auto slot=LocoSlot::getSlot(abs(p[0]),false);
    if (slot && slot->isConsistLead()) {
        DCCConsist::deleteAnyConsist(p[0]);
    }
    if (params<2) return true; // we only had to delete 
     
    for (byte i=0;i<params;i++) {
        auto slot=LocoSlot::getSlot(abs(p[i]),false);
        if (slot && (slot->isConsistLead() || slot->isConsistFollower())) {
            StringFormatter::send(stream,F("<* Loco %d in another consist *>"),abs(p[i]));
            return false;
        }
    }
    
    auto leadLoco=LocoSlot::getSlot(abs(p[0]),true);
    LocoSlot *  prev=leadLoco;
    for (byte i=1;i<params;i++) {
        auto slot=LocoSlot::getSlot(abs(p[i]),true);
        slot->setConsistLead(leadLoco);
        slot->setConsistReverse(p[i]<0);
        prev->setConsistNext(slot);        
        prev=slot;
    }
    return true;
}

void DCCConsist::deleteAnyConsist(int16_t locoid) {
    locoid=abs(locoid); // in case of (valid) negative
    auto slot=LocoSlot::getSlot(locoid,false);
    if (!slot) return; // no such loco, nothing to do
    if (!slot->isConsistLead()) slot=slot->getConsistLead();
    if (!slot) return; // not in consist, nothing to do

    while(slot) {
      auto next=slot->getConsistNext();
      slot->setConsistLead(nullptr);
      slot->setConsistNext(nullptr);
      slot->setConsistReverse(false);
      slot=next;
    } 
}

bool DCCConsist::addLocoToConsist(uint16_t consistId,uint16_t locoId, bool revesed) {
    if (consistId<1 || consistId>10239) return false;
    if (locoId<1 || locoId>10239) return false;
    if (locoId==consistId) return true; // cant add lead to itself
    
    auto leadSlot=LocoSlot::getSlot(consistId,true);
    if (!leadSlot) return false; // no ram
    // back up to lead of existing consist
    while(leadSlot->getConsistLead()) {
        leadSlot=leadSlot->getConsistLead();
    }

    // find tail of consist
    auto tailSlot=leadSlot;
    while(tailSlot->getConsistNext()) {
        tailSlot=tailSlot->getConsistNext();
    }

    auto addSlot=LocoSlot::getSlot(locoId,true);
    if (!addSlot) return false; // no ram
    
    if (addSlot->isConsistLead() || addSlot->isConsistFollower()) {
        DIAG(F("Loco %d already in a consist"),locoId);
        return false; // already in consist
    }
    
    // All OK so chain in. (belt and braces prevent self reference loop)
    addSlot->setConsistReverse(revesed);
    if (addSlot!=leadSlot) addSlot->setConsistLead(leadSlot);    
    if (tailSlot!=addSlot) tailSlot->setConsistNext(addSlot);
    return true;
}
 
