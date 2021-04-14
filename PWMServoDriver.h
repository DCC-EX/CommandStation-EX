/*
 *  (c) 2020 Chris Harlow. All rights reserved.
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
/*!
 *  @file PWMServoDriver.h
 *
 *  Used to set servo positions on an I2C bus with 1 or more PCA96685 boards.
 */
#ifndef PWMServoDriver_H
#define PWMServoDriver_H


class PWMServoDriver {
public:
    static void setServo(byte servoNum,  uint16_t pos);
    
private:
  static byte setupFlags; 
  static byte failFlags; 
  static bool setup(int board);
  static void writeRegister(uint8_t i2caddr,uint8_t hardwareRegister, uint8_t d);
};

#endif
