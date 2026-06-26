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

 // Node traffic command parsing.

/* These are the commands that are sent by nodes to  each other.
  They represent calls for a state change so all nodes keep in sync.
  If the receiver is the guardian of the actual hardware involved (
  ie is the CS gereating waveforms for a dcc state or the driver of a turnout servo) then
  this node will just do the job.
  State changes coming in this way are NOT rebroadcast so the majority of functions will have the
  non-default value false for the tellNodes parameter.

  The ZZPARSER technology is used here for convenience.  This is, however a completely different
   set of commands to those between throttle and CS and can overlap opcodes and formats since they are
   never mixed.
  */

ZZBEGIN
ZZ(t,loco,speedByte)  // Throttle speed change
  // This was sent by any node that changes a loco speed
  // It will not cause a rebroadcast
  DCC::getLoco(loco)->setTargetSpeed(speedByte,false);
ZZ(F,loco,functionNumber,on)  // Throttle function change
  // This was sent by any node that changes a loco function
  // It will not cause a rebroadcast
  DCC::setFn(functionNumber,on,false);

ZZ(H,turnoutid,bit)  // Turnout throw/close (1=thrown, 0=closed)
  // This was sent by a node that changes a turnout state
  Turnout::setClosed(turnoutid,bit==0,false);

ZZ(S,signalid,rag) // Signal aspect change (R=red, A=amber, G=green)
  Signal::setSignal(signalid,(Signal::RAG)rag,false);   

ZZEND
