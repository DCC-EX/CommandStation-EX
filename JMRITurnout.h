#ifndef JMRITurnout_h
#define TMRITurnout_h

#include <Arduino.h>
#include "Config.h"

class JMRITurnout {

  public:
  static void parse(Stream & stream, int params, int p[]);

  
  
  private:

  static void showAll(Stream & stream);
  static bool show(Stream & stream ,int id, bool all);
  static bool create(int id, int address, byte subAddress);
  static bool remove(int id);
  static bool activate(int id, bool thrown);
  static bool invalid(int id);

  struct  TurnoutEntry {
         int address;
         byte subAddress;
         bool thrown;
  };
  static TurnoutEntry table[MAX_TURNOUTS];
};
#endif
