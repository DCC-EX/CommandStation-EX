#ifndef Turnouts_h
#define Turnouts_h

#include <Arduino.h>
#include "DCC.h"

struct TurnoutData {
  uint8_t tStatus;
  uint8_t subAddress;
  int id;
  int address;  
};

struct Turnout{
  static Turnout *firstTurnout;
  int num;
  struct TurnoutData data;
  Turnout *nextTurnout;
  static bool activate(int n, bool state);
  static Turnout* get(int);
  static bool remove(int);
  static void load();
  static void store();
  static Turnout *create(int, int, int);
  static void show(Stream & stream, int n);
  static void showAll(Stream & stream);
}; // Turnout
  
#endif
