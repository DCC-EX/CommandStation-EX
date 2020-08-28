/*
 *  BoardWSMFireBoxMK1T.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BOARDS_BOARDWSMFIREBOXMK1A_H_
#define BOARDS_BOARDWSMFIREBOXMK1A_H_

#include "Board.h"

#if defined(ARDUINO_ARCH_SAMD)

struct BoardConfigWSMFireBoxMK1A : public BoardConfig {
  
};

class BoardWSMFireBoxMK1A : public Board
{
public:
  BoardConfigWSMFireBoxMK1A config;

  BoardWSMFireBoxMK1A(BoardConfigWSMFireBoxMK1A _config) {
    config = _config;
  }

  static void getDefaultConfigA(BoardConfigWSMFireBoxMK1A& _config) {
    _config.track_name = "A";
    _config.signal_a_pin = 6;
    _config.signal_b_pin = 7;
    _config.enable_pin = 3;
    _config.sense_pin = A5;   
    _config.board_voltage = 3.3;
    _config.amps_per_volt = 1.998004;
    _config.current_trip = 5000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file

    _config.railcom_baud = 250000;

    _config.prog_threshold = 30;
  }

  static void getDefaultConfigB(BoardConfigWSMFireBoxMK1A& _config) {
    _config.track_name = "B";
    _config.signal_a_pin = 8;
    _config.signal_b_pin = 9;
    _config.enable_pin = 4;
    _config.sense_pin = A1;       
    _config.board_voltage = 3.3;
    _config.amps_per_volt = 1.998004;
    _config.current_trip = 5000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file

    _config.railcom_baud = 250000;

    _config.prog_threshold = 30;
  }

  void setup();

  const char* getName();
  
  void power(bool, bool announce);
  void signal(bool);
  void cutout(bool);
  void progMode(bool);

  uint16_t getCurrentRaw();
  uint16_t getCurrentMilliamps();
  uint16_t getCurrentMilliamps(uint16_t reading);

  uint16_t setCurrentBase();
  uint16_t getCurrentBase();

  bool getStatus();

  void checkOverload();

  uint8_t getPreambles();

  void rcomCutout(bool);
  void rcomEnable(bool) {}
  void rcomRead() {}

  uint16_t getThreshold() { return config.prog_threshold; }
  
private:
  bool isCurrentLimiting();

  // Variable used to stop track power from coming on during cutout
  bool inCutout;
};

#endif  // ARDUINO_ARCH_SAMD

#endif  // BOARDS_BOARDWSMFIREBOXMK1A_H_