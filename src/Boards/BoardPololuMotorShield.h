/*
 *  BoardPololuMotorShield.h
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

#ifndef BOARDS_BOARDPOLOLUMOTORSHIELD_H_
#define BOARDS_BOARDPOLOLUMOTORSHIELD_H_

#include "Board.h"

struct BoardConfigPololuMotorShield : public BoardConfig {
  // Used to store functionality special to this type of shield
};

class BoardPololuMotorShield : public Board
{
public:
  BoardConfigPololuMotorShield config;

  BoardPololuMotorShield(BoardConfigPololuMotorShield _config) {
    config = _config;
  }

  static void getDefaultConfigA(BoardConfigPololuMotorShield& _config) {
    _config.track_name = "A";
    _config.signal_a_pin = 7;
    _config.signal_b_pin = 9;
    _config.enable_pin = 4;
    _config.sense_pin = A0;   
    _config.board_voltage = 5.0;
    _config.amps_per_volt = 1.904762;
    _config.current_trip = 3000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file
    _config.prog_threshold = 30;
  }

  static void getDefaultConfigB(BoardConfigPololuMotorShield& _config) {
    _config.track_name = "B";
    _config.signal_a_pin = 8;
    _config.signal_b_pin = 10;
    _config.enable_pin = 4;
    _config.sense_pin = A1;       
    _config.board_voltage = 5.0;
    _config.amps_per_volt = 1.904762;
    _config.current_trip = 3000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file
    _config.prog_threshold = 30;
  }

  void setup();

  const char* getName();
  
  void power(bool, bool announce);
  void signal(bool);
  void progMode(bool);

  uint16_t getCurrentRaw();
  uint16_t getCurrentMilliamps();
  uint16_t getCurrentMilliamps(uint16_t reading);

  uint16_t setCurrentBase();
  uint16_t getCurrentBase();

  bool getStatus();

  void checkOverload();

  uint8_t getPreambles();

  uint16_t getThreshold() { return config.prog_threshold; }
  
  void rcomCutout(bool);
  
  void rcomEnable(bool) {} // Empty because not supported on this board
  void rcomRead() {} // Empty because not supported on this board
private:
  bool isCurrentLimiting();
};


#endif  // BOARDS_BOARDPOLOLUMOTORSHIELD_H_