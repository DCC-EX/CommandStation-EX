/*
 *  © 2023 Peter Cole
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


/*
 * Public static data
 */
int Turntable::turntablelistHash = 0;


/*
 * Protected static functions
 */
// Add new turntable to end of list

void Turntable::add(Turntable *tto) {
  if (!_firstTurntable) {
    _firstTurntable = tto;
  } else {
    Turntable *ptr = _firstTurntable;
    for ( ; ptr->_nextTurntable!=0; ptr=ptr->_nextTurntable) {}
    ptr->_nextTurntable = tto;
  }
  turntablelistHash++;
}

// Find turntable from list
Turntable *Turntable::get(uint16_t id) {
  for (Turntable *tto = _firstTurntable; tto != NULL; tto = tto->_nextTurntable)
    if (tto->_turntableData.id == id) return tto;
  return NULL;
}

// Remove specified turntable from list and delete it
bool Turntable::remove(uint16_t id) {
  Turntable *tto, *pp=NULL;

  for (tto=_firstTurntable; tto!=NULL && tto->_turntableData.id!=id; pp=tto, tto=tto->_nextTurntable) {}
  if (tto == NULL) return false;
  if (tto == _firstTurntable) {
    _firstTurntable = tto->_nextTurntable;
  } else {
    pp->_nextTurntable = tto->_nextTurntable;
  }

  delete (EXTTTurntable *)tto;

  turntablelistHash++;
  return true;
}

/*
 * Public static functions
 */
bool Turntable::isPosition(uint16_t id, uint8_t position) {
  Turntable *tto = get(id);
  if (tto) {
    if (tto->getPosition() == position) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool Turntable::setPositionStateOnly(uint16_t id, uint8_t position) {
  Turntable *tto = get(id);
  if (!tto) return false;
  CommandDistributor::broadcastTurntable(id, position);
#if defined(EXRAIL_ACTIVE)
  // RMFT2::turntableEvent(id, position);
#endif
  return true;
}

bool Turntable::setPosition(uint16_t id, uint8_t position, uint8_t activity) {
#if defined(DIAG_IO)
  DIAG(F("Turntable(%d, %d)"), id, position);
#endif
  Turntable *tto = Turntable::get(id);
  if (!tto) return false;
  bool ok = tto->setPositionInternal(position, activity);

  if (ok) {
    tto->setPositionStateOnly(id, position);
  }
  return ok;
}

/*************************************************************************************
 * EXTTTurntable - EX-Turntable device.
 * 
 *************************************************************************************/
// Private constructor
EXTTTurntable::EXTTTurntable(uint16_t id, uint8_t i2caddress, VPIN vpin) :
  Turntable(id, TURNTABLE_EXTT)
{
  _exttTurntableData.i2caddress = i2caddress;
  _exttTurntableData.vpin = vpin;
}

// Create function
#ifndef IO_NO_HAL
  Turntable *EXTTTurntable::create(uint16_t id, uint8_t i2caddress, VPIN vpin) {
    Turntable *tto = get(id);
    if (tto) {
      if (tto->isType(TURNTABLE_EXTT)) {
        EXTTTurntable *extt = (EXTTTurntable *)tto;
        extt->_exttTurntableData.i2caddress = i2caddress;
        extt->_exttTurntableData.vpin = vpin;
        return tto;
      } else {
        remove(id);
      }
    }
    tto = (Turntable *)new EXTTTurntable(id, i2caddress, vpin);
    DIAG(F("Turntable 0x%x"), tto);
    return tto;
#else
  (void)id;
  (void)i2caddress;
  (void)vpin;
  (void)positions;
  return NULL;
#endif
  }

  void EXTTTurntable::print(Print *stream) {
    StringFormatter::send(stream, F("<i %d EXTURNTABLE %d %d>\n"), _turntableData.id, _exttTurntableData.i2caddress, _exttTurntableData.vpin);
  }

  // EX-Turntable specific code for moving to the specified position
  bool EXTTTurntable::setPositionInternal(uint8_t position, uint8_t activity) {
#ifndef IO_NO_HAL
    // Get step value from positions
    // int value = _exttTurntableData.positions[position];
    // Set position via device driver
    // EXTurntable::_writeAnalogue(vpin, value, activity, 0);
#else
    (void)position;
#endif
    return true;
  }
