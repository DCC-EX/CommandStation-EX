/* Copyright (c) 2023 Harald Barth
 *
 * This source is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This source is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>

#include "DCC.h" // fetch MAX_LOCOS from there

class LocoTable {
public:
  void forgetLoco(int cab) {
    int reg=lookupSpeedTable(cab, false);
    if (reg>=0) speedTable[reg].loco=0;
  }
  static int lookupSpeedTable(int locoId, bool autoCreate);
  static bool updateLoco(int loco, byte speedCode);
  static bool updateFunc(int loco, byte func, int shift);
  static void dumpTable(Stream *output);

private:
  struct LOCO
  {
    int loco;
    byte speedCode;
    byte groupFlags;
    unsigned long functions;
    unsigned int funccounter;
    unsigned int speedcounter;
  };
  static LOCO speedTable[MAX_LOCOS];
  static int highestUsedReg;
};
