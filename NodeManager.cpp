/*
 *  © 2026 Chris Harlow
 *  All rights reserved.
 *  
 *  This file is part of DCC-EX
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
#include "NodeManager.h"
#include "DCCEXParser.h"
#include "NetworkInterface.h"
#include "StringFormatter.h"
#include "DIAG.h"
#include "Turnouts.h"


bool NodeManager::enabled = false;
void NodeManager::setup(bool enabledFlag) {
    enabled = enabledFlag;
}

void NodeManager::cast(const FSH* format...) {
    if (!enabled) return;
    StringBuffer buffer(260);
    va_list args;
    va_start(args, format);
    StringFormatter::send2(&buffer, format, args);
    va_end(args);
    cast(&buffer);
}

void NodeManager::cast(StringBuffer *buffer) {
    if (!enabled || buffer == nullptr || buffer->getLength() <= 0) return;
    NetworkInterface::udpNodeMulticast(buffer->getString());
    if (Diag::NODE) DIAG(F("Node out: %s"), buffer->getString());
}

void NodeManager::castVpin(VPIN vpin, int16_t count, int16_t value) {
    if (!enabled || !IODevice::isSharedWrite(vpin, count)) return;
    StringBuffer buffer(128);
    StringFormatter::send(&buffer, F("<z %d %d %d>"), vpin, value, count);
    cast(&buffer);
}

void NodeManager::castVpin(VPIN vpin, int16_t count, int16_t value,
                           int16_t param1, int16_t param2) {
    if (!enabled || !IODevice::isSharedWrite(vpin, count)) return;
    StringBuffer buffer(128);
    StringFormatter::send(&buffer, F("<z %d %d %d %d %d>"),
                          vpin, value, (uint16_t)param1, param2, count);
    cast(&buffer);
}

void NodeManager::parse(byte *cmd) {
    if (Diag::NODE) DIAG(F("Node in: %s"), cmd);
    DCCEXParser::parseNodeTraffic(cmd);
}

