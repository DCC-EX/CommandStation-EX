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

#ifndef BOARDS_BOARDWSMFIREBOXMK1T_H_
#define BOARDS_BOARDWSMFIREBOXMK1T_H_

#include "Board.h"

#if defined(ARDUINO_ARCH_SAMC)

struct BoardConfigWSMFireBoxMK1T : public BoardConfig {
  // Used to store functionality special to this type of shield
  int cutout_pin;
  int limit_pin;

  Uart* serial;
};

class BoardWSMFireBoxMK1T : public Board
{
public:
  BoardConfigWSMFireBoxMK1T config;

  BoardWSMFireBoxMK1T(BoardConfigWSMFireBoxMK1T _config) {
    config = _config;
  }

  static void getDefaultConfigA(BoardConfigWSMFireBoxMK1T& _config) {
    _config.track_name = "A";
    _config.signal_a_pin = 20;
    _config.signal_b_pin = 21;
    _config.enable_pin = 23;
    _config.sense_pin = 22;   
    _config.board_voltage = 3.3;
    _config.amps_per_volt = 0.6;
    _config.current_trip = 5000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file

    _config.cutout_pin = 32;
    _config.limit_pin = 25;

    _config.serial = &Serial2;
    _config.railcom_baud = 250000;

    _config.prog_threshold = 30;
  }

  static void getDefaultConfigB(BoardConfigWSMFireBoxMK1T& _config) {
    _config.track_name = "B";
    _config.signal_a_pin = 26;
    _config.signal_b_pin = 27;
    _config.enable_pin = 29;
    _config.sense_pin = 28;       
    _config.board_voltage = 3.3;
    _config.amps_per_volt = 0.6;
    _config.current_trip = 5000;
    _config.current_trip_prog = 250;
    _config.prog_trip_time = 100;
    _config.main_preambles = 16;
    _config.prog_preambles = 22;
    _config.track_power_callback = nullptr; // Needs to be set in the main file
  
    _config.cutout_pin = 34;
    _config.limit_pin = 31;

    _config.serial = &Serial3;
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
  void rcomEnable(bool);
  void rcomRead();

  uint16_t getThreshold() { return config.prog_threshold; }
  
private:
  bool isCurrentLimiting();

  // Variable used to stop track power from coming on during cutout
  bool inCutout;
};

#endif  // ARDUINO_ARCH_SAMC

#endif  // BOARDS_BOARDWSMFIREBOXMK1T_H_