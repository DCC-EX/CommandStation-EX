/*
 *  © 2024, Chris Harlow. All rights reserved.
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
/* 
This is the list of HAL drivers automatically included by IODevice.h
It has been moved here to be easier to maintain than editing IODevice.h
*/
#include "IO_AnalogueInputs.h"
#include "IO_DFPlayer.h"
#include "IO_DS1307.h"
#include "IO_duinoNodes.h"
#include "IO_EncoderThrottle.h"
#include "IO_EXFastclock.h"
#include "IO_EXIOExpander.h"
#include "IO_EXSensorCAM.h"
#include "IO_HALDisplay.h"
#include "IO_HCSR04.h"
#include "IO_I2CDFPlayer.h"
#include "IO_I2CRailcom.h"
#include "IO_MCP23008.h"
#include "IO_MCP23017.h"
#include "IO_NeoPixel.h"
#include "IO_PCA9555.h"
#include "IO_PCA9685pwm.h"
#include "IO_PCF8574.h"
#include "IO_PCF8575.h"
#include "IO_RotaryEncoder.h"
#include "IO_Servo.h"
#include "IO_TCA8418.h"
#include "IO_TM1638.h"
#include "IO_TouchKeypad.h"
#include "IO_trainbrains.h"
#include "IO_Bitmap.h"
#include "IO_VL53L0X.h"

