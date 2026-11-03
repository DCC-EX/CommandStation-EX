/*
 *  © 2023 Peter Cole
 *  © 2026 Chris Harlow
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

#include "defines.h"
#include <Arduino.h>
#include "Turntables.h"
#include "StringFormatter.h"
#include "CommandDistributor.h"
#include "EXRAIL2.h"
#include "DCC.h"

/*
 * Protected static data
 */
Turntable *Turntable::_firstTurntable = 0;

Turntable::Turntable() {}

Turntable* Turntable::create(uint16_t id, TurntableType turntableType,uint16_t homeAngle,VPIN extVpin, const char * description) {
    Turntable *tt = get(id);
    if (tt) {
      DIAG(F("Duplicate Turntable ID: %d\n"), id);
      return nullptr;
    }
    tt = new Turntable();
    if (!tt) return tt;
    tt->id = id;
    tt->type = turntableType;
    tt->moving = false;
    tt->position = 0;
    tt->previousPosition = 0;
    tt->extVpin = extVpin;
    tt->homeAngle=homeAngle;
    tt->description = new char[strlen(description) + 1];
    strcpy(tt->description, description);
    tt->_firstPosition = nullptr; 
    tt->addPosition(0,0, homeAngle, "Home");
    tt->_nextTurntable = _firstTurntable;
    _firstTurntable = tt;
    return tt;
  }

  TurntablePosition* Turntable::addPosition(uint8_t idx, uint16_t value, uint16_t angle, const char * description) {
    for (auto p=_firstPosition;p;p=p->next) {
      if (p->index==idx) {
        DIAG(F("Duplicate Turntable %d Position Index: %d\n"), id, idx);
        return nullptr;
      }
    }
    TurntablePosition* newPosition = new TurntablePosition();
    newPosition->index = idx;
    newPosition->dccAddress = value; // same for DCC and EXTT, union handles it
    newPosition->angle = angle;
    newPosition->description = new char[strlen(description) + 1];
    strcpy(newPosition->description, description);
    newPosition->next = _firstPosition;
    _firstPosition = newPosition;
    return newPosition;
  }


/*
 * Protected static functions
 */

/*
 * Public static functions
 */
// Find turntable from list
Turntable *Turntable::get(uint16_t id) {
  for (auto *tto = _firstTurntable; tto != nullptr; tto = tto->_nextTurntable)
    if (tto->id == id) return tto;
  return nullptr;
}

uint16_t Turntable::getByVpin(VPIN vpin) {
  for (auto *tto = _firstTurntable; tto != nullptr; tto = tto->_nextTurntable)
    if (tto->type==TURNTABLE_EXTT && tto->extVpin == vpin) return tto->id;
  return 0;
}

uint8_t Turntable::getPositionCount() {
      byte count=0;
      for (auto p=_firstPosition; p; p=p->next) if (p->description[0]!=0x01) count++;
      return count;
}

// Get the current position for turntable with the specified ID
uint8_t Turntable::getPosition(uint16_t id) {
  Turntable *tto = get(id);
  if (!tto) return 0;
  return tto?tto->position:0;
}

// Got the moving state of the specified turntable
bool Turntable::isMoving(uint16_t id) {
  Turntable *tto = get(id);
  return tto?tto->moving:false;
}

void Turntable::setMoving(uint16_t id, bool isMoving, bool tellNodes) {
  auto *tto = Turntable::get(id);
  if (!tto) return;
  if (tto->moving == isMoving) return;
  tto->moving=isMoving;
  CommandDistributor::broadcastTurntable(id, tto->position, isMoving);
  if (tellNodes)NodeManager::cast(F("<i %d %b>"), id, isMoving);
}

// Initiate a turntable move (static)
bool Turntable::setPosition(uint16_t id, uint8_t position, ActivityNumber activity) {
  return setPosition(id, position, static_cast<byte>(activity));
}

bool Turntable::setPosition(uint16_t id, uint8_t position, byte activity, bool tellNodes) {
  auto *tto = Turntable::get(id);
  return tto?tto->setPositionInternal(position, activity, tellNodes):false;
}



  // EX-Turntable specific code for moving to the specified position
  bool Turntable::setPositionInternal(uint8_t newposition, byte activity, bool tellNodes) {
    if (newposition==0) { // send activity only }
      if (type==TURNTABLE_DCC) return false;  // DCC turntables cannot send activity only commands
      if (type==TURNTABLE_EXTT) IODevice::writeAnalogue(extVpin, 0, activity);
      if (tellNodes) NodeManager::cast(F("<i %d %d %d>"), id, 0, activity);
      return true;
    }
    
    if (activity != ActivityNumber::Turn && activity != ActivityNumber::Turn_PInvert) return false; // If sending a position update, only phase changes valid (0|1)
    if (position == newposition) return false; // No change in position
    
    // locate the positoion data
    auto pp=_firstPosition;
    while (pp && pp->index != newposition) pp = pp->next;
    if (!pp) return false; // Position not found
  
    previousPosition = position;
    if (type==TURNTABLE_EXTT) IODevice::writeAnalogue(extVpin, pp->exttData, activity);
    if (type==TURNTABLE_DCC) DCC::setAccessory(pp->dccAddress>>3, pp->dccAddress & 0x03, 1);
    if (tellNodes) NodeManager::cast(F("<i %d %d %d>"), id, newposition, activity);
    CommandDistributor::broadcastTurntable(id, newposition, true); // is moving
    return true;
  }

  void Turntable::printState(Print *stream) {
    switch(type) {
      case TURNTABLE_EXTT:
        StringFormatter::send(stream, F("<I %d EXT %d %d>\n"), id, extVpin, homeAngle);
        break;
      case TURNTABLE_DCC:
        StringFormatter::send(stream, F("<I %d DCC %d>\n"), id, homeAngle);
        break;
      case TURNTABLE_REMOTE:
        StringFormatter::send(stream, F("<I %d REMOTE>\n"), id, getPosition());
        break;
    }
    for (auto pp=_firstPosition; pp != nullptr; pp = pp->next)
      StringFormatter::send(stream, F("<I %d ADD %d %d %d>\n"), id, pp->index, pp->exttData, pp->angle);
  }

 void Turntable::printAll(Print *stream) {
    for (auto *tto = _firstTurntable; tto != 0; tto = tto->_nextTurntable) tto->printState(stream);
  }
void Turntable::shareNodesToCS() {
    for (auto *tto = _firstTurntable; tto != 0; tto = tto->_nextTurntable) tto->shareNodeToCS();
  }

  void Turntable::shareNodeToCS() {
    if (type==TURNTABLE_REMOTE) return;
    NodeManager::cast(F("<I %d %d \"%s\">"), id,homeAngle, description);
    for (auto pp=_firstPosition; pp != nullptr; pp = pp->next)
      if (pp->index)NodeManager::cast(F("<I %d %d %d \"%s\">"), id, pp->index, pp->angle, pp->description);
    NodeManager::cast(F("<i %d %d %d>"), id, position, moving);  
  }
  