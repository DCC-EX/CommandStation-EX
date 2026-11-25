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
#include "EXNetwork.h"
#include "StringFormatter.h"
#include "DIAG.h"
#include "Turnouts.h"
#include "EXNetworkPreferences.h"
#include "Sensors.h"
#include "Turntables.h"
#include "Signals.h"
#include "CommandDistributor.h"



bool NodeManager::enabled = false;
OtherNode * NodeManager::firstOtherNode = nullptr;

void NodeManager::addOtherNode(const char * name, const char * ipAddress) {
    // if we receive a new node after we have gone through the node sharing phase,
    // all bets afre off and everything will start by sharing again.
    
    if(sodState > SOD_NAMES_SHARED) {
        OtherNode * nexone = nullptr;
        for (OtherNode * node = firstOtherNode; node != nullptr; node = nexone) {
            nexone = node->next;
            delete node;
        }
        firstOtherNode = nullptr;

        // reset SOD sequence to share state again
        sodState = SOD_WAITING_TO_START;
        sodTime = 0;
    }

    // add node to list
    for (OtherNode * node = firstOtherNode; node != nullptr; node = node->next) {
        if (strcmp(node->name, name) == 0 && strcmp(node->ipAddress, ipAddress) == 0) {
            return; // Node already exists
        }
    }

    OtherNode * node = new OtherNode();
    strncpy(node->name, name, sizeof(node->name) - 1);
    node->name[sizeof(node->name) - 1] = '\0';
    strncpy(node->ipAddress, ipAddress, sizeof(node->ipAddress) - 1);
    node->ipAddress[sizeof(node->ipAddress) - 1] = '\0';
    node->next = firstOtherNode;
    firstOtherNode = node;

}

void NodeManager::shareNodeNameToCS() {
    cast(F("<N \"%s\" \"%d.%d.%d.%d\">"), 
         EXNetworkPreferences::getHostName(),
         EXNetwork::getMyIPAddress()[0], EXNetwork::getMyIPAddress()[1],
         EXNetwork::getMyIPAddress()[2], EXNetwork::getMyIPAddress()[3]);
}


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
    EXNetwork::udpNodeMulticast(buffer->getString());
    // diag will be done by udp queue to show outgoing merged messages 
}

void NodeManager::castSensorChange(char cmd,VPIN vpin) {
    if (!enabled) return;
    StringBuffer buffer(32);
    StringFormatter::send(&buffer, F("<%c %d>"), cmd,vpin);
    cast(&buffer);
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


/* Start Of Day (SOD) state and timing variables
   used to manage the sequence of actions at the start of the day.
   This code is used to allow all nodes to advertise their presence
   by calling shareNodeNameToCS() almost immediately after startup. 
   Nothing further happens until a significant delay has been reached to allow all nodes to be ready.

   Then each node is able to share its state using a slightly randomized and delayed sequence.
   this means that all nodes starting at roughly the same time will be able to share with each other without repetition.

   Should another node start after the initial sequence, its initial sharteNodeNameToCS() can be detected 
   and used to reset the SOD state of every started node. 
   Thus all previous nodes will share their state again with the new node.
   
*/

SODState NodeManager::sodState = SOD_WAITING_TO_START;
unsigned long NodeManager::sodTime = 0;


void NodeManager::loop() {
    if (sodState == SOD_DONE) return;
    if (millis()<sodTime) return;

    switch(sodState) {
        case SOD_WAITING_TO_START:
            sodTime=millis()+micros()%1000; // do it soon but randomize a bit to avoid collisions
            sodState=SOD_SHARE_HOSTNAME;
            break;
        case SOD_SHARE_HOSTNAME:
            shareNodeNameToCS();
            sodTime = millis() + 4000; // allow all starting nlodes to get past this step
            sodState = SOD_NAMES_SHARED;
            break;
        case SOD_NAMES_SHARED:
            // get here after sturtup hello phase sharing node names
            sodState = SOD_SHARE_TURNOUTS;
            break;
        case SOD_SHARE_TURNOUTS:
            Turnout::shareNodesToCS();
            sodTime = millis() + 1000;
            sodState = SOD_SHARE_SENSORS;
            break;
        case SOD_SHARE_SENSORS:
            Sensor::shareNodesToCS();
            sodTime = millis() + 1000;
            sodState = SOD_SHARE_SIGNALS;
            break;
        case SOD_SHARE_SIGNALS:
            Signal::shareNodesToCS();
            sodTime = millis() + 1000;
            sodState = SOD_SHARE_TURNTABLES;
            break;
        case SOD_SHARE_TURNTABLES:
            Turntable::shareNodesToCS();
            sodTime = millis() + 1000;
            sodState = SOD_SHARE_DISPLAYS;
            break;
        case SOD_SHARE_DISPLAYS:
            Display::shareNodesToCS();
            sodTime = millis() + 1000;
            sodState = SOD_BROADCAST_POWER;
            break;
        case SOD_BROADCAST_POWER:
            CommandDistributor::broadcastPower();
            sodTime = millis() + 1000;
            sodState = SOD_DONE;
            break;
        case SOD_DONE:
            // Nothing to do... see top of function!
            break;
    }
}
