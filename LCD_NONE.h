/*
 *  © 2021, Chris Harlow, Neil McKechnie. All rights reserved.
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

// dummy LCD shim to keep linker happy
  LCDDisplay::LCDDisplay() {} 
  void LCDDisplay::interfake(int p1, int p2, int p3) {(void)p1; (void)p2; (void)p3;}   
  void LCDDisplay::setRowNative(byte row) { (void)row;} 
  void LCDDisplay::clearNative() {}
  void LCDDisplay::writeNative(char b){ (void)b;} //  
  void LCDDisplay::displayNative(){}
  
