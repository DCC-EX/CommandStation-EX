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
 *  @file PWMServoDriver.cpp
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver, based on Adafruit_PWMServoDriver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *  This class contains a very small subset of the Adafruit version which
 *  is relevant to driving simple servos at 50Hz through a number of chained 
 *  servo driver boards (ie servos 0-15 on board 0x40, 16-31 on board 0x41 etc.)
 *  
 *  @section author Author
 *  Chris Harlow (TPL) 
 *
 */
#include <Arduino.h>
#include "PWMServoDriver.h"
#include "DIAG.h"
#include "I2CManager.h"


// REGISTER ADDRESSES
const byte PCA9685_MODE1=0x00;      // Mode Register 
const byte PCA9685_FIRST_SERVO=0x06;  /** low byte first servo register ON*/
const byte PCA9685_PRESCALE=0xFE;     /** Prescale register for PWM output frequency */
// MODE1 bits
const byte MODE1_SLEEP=0x10;   /**< Low power mode. Oscillator off */
const byte MODE1_AI=0x20;      /**< Auto-Increment enabled */
const byte MODE1_RESTART=0x80; /**< Restart enabled */

const byte PCA9685_I2C_ADDRESS=0x40;      /** First PCA9685 I2C Slave Address */
const float FREQUENCY_OSCILLATOR=25000000.0; /** Accurate enough for our purposes  */
const uint8_t PRESCALE_50HZ = (uint8_t)(((FREQUENCY_OSCILLATOR / (50.0 * 4096.0)) + 0.5) - 1);
const uint32_t MAX_I2C_SPEED = 1000000L; // PCA9685 rated up to 1MHz I2C clock speed
  
/*!
 *  @brief  Sets the PWM frequency for a chip to 50Hz for servos
 */

byte PWMServoDriver::setupFlags=0;  // boards that have been initialised
byte PWMServoDriver::failFlags=0;  // boards that have faild initialisation

bool PWMServoDriver::setup(int board) {
  if (board>3 || (failFlags & (1<<board))) return false; 
  if (setupFlags & (1<<board)) return true; 
  
  I2CManager.begin();
  I2CManager.setClock(MAX_I2C_SPEED);
  
  uint8_t i2caddr=PCA9685_I2C_ADDRESS + board;

  // Test if device is available
  byte error = I2CManager.checkAddress(i2caddr);
  if (error) {
    DIAG(F("I2C Servo device 0x%x Not Found %d"),i2caddr, error);
    failFlags|=1<<board;  
    return false;
  }
    
  //DIAG(F("PWMServoDriver::setup %x prescale=%d"),i2caddr,PRESCALE_50HZ); 
  writeRegister(i2caddr,PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
  writeRegister(i2caddr,PCA9685_PRESCALE, PRESCALE_50HZ);  
  writeRegister(i2caddr,PCA9685_MODE1,MODE1_AI);
  writeRegister(i2caddr,PCA9685_MODE1,  MODE1_RESTART | MODE1_AI);
  setupFlags|=1<<board;
  return true;
}

/*!
 *  @brief  Sets the PWM output to a servo
 */
void PWMServoDriver::setServo(byte servoNum, uint16_t value) {
  int board=servoNum/16; 
  int pin=servoNum%16;
  
  if (setup(board)) {
    DIAG(F("SetServo %d %d"),servoNum,value);  
    uint8_t buffer[] = {(uint8_t)(PCA9685_FIRST_SERVO + 4 * pin), // 4 registers per pin
      0, 0, (uint8_t)(value & 0xff), (uint8_t)(value >> 8)};
    if (value == 4095) buffer[2] = 0x10;   // Full on
    byte error=I2CManager.write(PCA9685_I2C_ADDRESS + board, buffer, sizeof(buffer));
    if (error!=0) DIAG(F("SetServo error %d"),error); 
  }
}

void PWMServoDriver::writeRegister(uint8_t i2caddr,uint8_t hardwareRegister, uint8_t d) {
  I2CManager.write(i2caddr, 2, hardwareRegister, d);
}
