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

// No turntable support without HAL
#ifndef IO_NO_HAL

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

// Add a position
void Turntable::addPosition(uint8_t idx, uint16_t value, uint16_t angle) {
  _turntablePositions.insert(idx, value, angle);
}

// Get value for position
uint16_t Turntable::getPositionValue(uint8_t position) {
  TurntablePosition* currentPosition = _turntablePositions.getHead();
  while (currentPosition) {
    if (currentPosition->index == position) {
      return currentPosition->data;
    }
    currentPosition = currentPosition->next;
  }
  return false;
}

// Get value for position
uint16_t Turntable::getPositionAngle(uint8_t position) {
  TurntablePosition* currentPosition = _turntablePositions.getHead();
  while (currentPosition) {
    if (currentPosition->index == position) {
      return currentPosition->angle;
    }
    currentPosition = currentPosition->next;
  }
  return false;
}

// Get the count of positions associated with the turntable
uint8_t Turntable::getPositionCount()  {
  TurntablePosition* currentPosition = _turntablePositions.getHead();
  uint8_t count = 0;
  while (currentPosition) {
    count++;
    currentPosition = currentPosition->next;
  }
  return count;
}

/*
 * Public static functions
 */
// Find turntable from list
Turntable *Turntable::get(uint16_t id) {
  for (Turntable *tto = _firstTurntable; tto != nullptr; tto = tto->_nextTurntable)
    if (tto->_turntableData.id == id) return tto;
  return NULL;
}

// Find turntable via Vpin
Turntable *Turntable::getByVpin(VPIN vpin) {
  for (Turntable *tto = _firstTurntable; tto != nullptr; tto = tto->_nextTurntable) {
    if (tto->isEXTT()) {
      EXTTTurntable *exttTto = static_cast<EXTTTurntable*>(tto);
      if (exttTto->getVpin() == vpin) {
        return tto;
      }
    }
  }
  return nullptr;
}

// Get the current position for turntable with the specified ID
uint8_t Turntable::getPosition(uint16_t id) {
  Turntable *tto = get(id);
  if (!tto) return false;
  return tto->getPosition();
}

// Got the moving state of the specified turntable
bool Turntable::ttMoving(uint16_t id) {
  Turntable *tto = get(id);
  if (!tto) return false;
  return tto->isMoving();
}

// Initiate a turntable move
bool Turntable::setPosition(uint16_t id, uint8_t position, uint8_t activity) {
#if defined(DIAG_IO)
  DIAG(F("Rotate turntable %d to position %d, activity %d)"), id, position, activity);
#endif
  Turntable *tto = Turntable::get(id);
  if (!tto) return false;
  if (tto->isMoving()) return false;
  bool ok = tto->setPositionInternal(position, activity);

  if (ok) {
    // We only deal with broadcasts for DCC turntables here, EXTT in the device driver
    if (!tto->isEXTT()) {
      CommandDistributor::broadcastTurntable(id, position, false);
    }
    // Trigger EXRAIL rotateEvent for both types here if changed
#if defined(EXRAIL_ACTIVE)
    bool rotated = false;
    if (position != tto->_previousPosition) rotated = true;
    RMFT2::rotateEvent(id, rotated);
#endif
  }
  return ok;
}

/*************************************************************************************
 * EXTTTurntable - EX-Turntable device.
 * 
 *************************************************************************************/
// Private constructor
EXTTTurntable::EXTTTurntable(uint16_t id, VPIN vpin) :
  Turntable(id, TURNTABLE_EXTT)
{
  _exttTurntableData.vpin = vpin;
}

using DevState = IODevice::DeviceStateEnum;

// Create function
  Turntable *EXTTTurntable::create(uint16_t id, VPIN vpin) {
#ifndef IO_NO_HAL
    Turntable *tto = get(id);
    if (tto) {
      if (tto->isType(TURNTABLE_EXTT)) {
        EXTTTurntable *extt = (EXTTTurntable *)tto;
        extt->_exttTurntableData.vpin = vpin;
        return tto;
      }
    }
    if (!IODevice::exists(vpin)) return nullptr;
    if (IODevice::getStatus(vpin) == DevState::DEVSTATE_FAILED) return nullptr;
    if (Turntable::getByVpin(vpin)) return nullptr;
    tto = (Turntable *)new EXTTTurntable(id, vpin);
    DIAG(F("Turntable 0x%x size %d size %d"), tto, sizeof(Turntable), sizeof(struct TurntableData));
    return tto;
#else
  (void)id;
  (void)vpin;
  return NULL;
#endif
  }

  void EXTTTurntable::print(Print *stream) {
    StringFormatter::send(stream, F("<i %d EXTURNTABLE %d>\n"), _turntableData.id, _exttTurntableData.vpin);
  }

  // EX-Turntable specific code for moving to the specified position
  bool EXTTTurntable::setPositionInternal(uint8_t position, uint8_t activity) {
#ifndef IO_NO_HAL
    int16_t value;
    if (position == 0) {
      value = 0;  // Position 0 is just to send activities
    } else {
      if (activity > 1) return false; // If sending a position update, only phase changes valid (0|1)
      value = getPositionValue(position); // Get position value from position list
    }
    if (position > 0 && !value) return false; // Return false if it's not a valid position
    // Set position via device driver
    _previousPosition = _turntableData.position;
    _turntableData.position = position;
    EXTurntable::writeAnalogue(_exttTurntableData.vpin, value, activity);
#else
    (void)position;
#endif
    return true;
  }

/*************************************************************************************
 * DCCTurntable - DCC Turntable device.
 * 
 *************************************************************************************/
// Private constructor
DCCTurntable::DCCTurntable(uint16_t id) : Turntable(id, TURNTABLE_DCC) {}

// Create function
  Turntable *DCCTurntable::create(uint16_t id) {
#ifndef IO_NO_HAL
    Turntable *tto = get(id);
    if (!tto) {
      tto = (Turntable *)new DCCTurntable(id);
      DIAG(F("Turntable 0x%x size %d size %d"), tto, sizeof(Turntable), sizeof(struct TurntableData));
    }
    return tto;
#else
  (void)id;
  return NULL;
#endif
  }

  void DCCTurntable::print(Print *stream) {
    StringFormatter::send(stream, F("<i %d DCCTURNTABLE>\n"), _turntableData.id);
  }

  // EX-Turntable specific code for moving to the specified position
  bool DCCTurntable::setPositionInternal(uint8_t position, uint8_t activity) {
#ifndef IO_NO_HAL
    int16_t value = getPositionValue(position);
    if (position == 0 || !value) return false; // Return false if it's not a valid position
    // Set position via device driver
    int16_t addr=value>>3;
    int16_t subaddr=(value>>1) & 0x03;
    bool active=value & 0x01;
    _previousPosition = _turntableData.position;
    _turntableData.position = position;
    DCC::setAccessory(addr, subaddr, active);
#else
    (void)position;
#endif
    return true;
  }

#endif
