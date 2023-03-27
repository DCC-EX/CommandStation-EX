/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
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

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"

// REGISTER ADDRESSES
static const byte PCA9685_MODE1=0x00;      // Mode Register 
static const byte PCA9685_FIRST_SERVO=0x06;  /** low byte first servo register ON*/
static const byte PCA9685_PRESCALE=0xFE;     /** Prescale register for PWM output frequency */
// MODE1 bits
static const byte MODE1_SLEEP=0x10;   /**< Low power mode. Oscillator off */
static const byte MODE1_AI=0x20;      /**< Auto-Increment enabled */
static const byte MODE1_RESTART=0x80; /**< Restart enabled */

static const float FREQUENCY_OSCILLATOR=25000000.0; /** Accurate enough for our purposes  */
static const uint32_t MAX_I2C_SPEED = 1000000L; // PCA9685 rated up to 1MHz I2C clock speed

// Predeclare helper function
static void writeRegister(byte address, byte reg, byte value);

// Create device driver instance.
void PCA9685::create(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint16_t frequency) {
  if (checkNoOverlap(firstVpin, nPins,i2cAddress)) new PCA9685(firstVpin, nPins, i2cAddress, frequency);
}

// Configure a port on the PCA9685.
bool PCA9685::_configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) {
  if (configType != CONFIGURE_SERVO) return false;
  if (paramCount != 5) return false;
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Configure VPIN:%u Apos:%d Ipos:%d Profile:%d Duration:%d state:%d"), 
    vpin, params[0], params[1], params[2], params[3], params[4]);
  #endif

  int8_t pin = vpin - _firstVpin;
  struct ServoData *s = _servoData[pin];
  if (s == NULL) { 
    _servoData[pin] = (struct ServoData *)calloc(1, sizeof(struct ServoData));
    s = _servoData[pin];
    if (!s) return false; // Check for failed memory allocation
  }

  s->activePosition = params[0];
  s->inactivePosition = params[1];
  s->profile = params[2];
  s->duration = params[3];
  int state = params[4];

  if (state != -1) {
    // Position servo to initial state
    _writeAnalogue(vpin, state ? s->activePosition : s->inactivePosition, 0, 0);
  } 
  return true;
}

// Constructor
PCA9685::PCA9685(VPIN firstVpin, int nPins, I2CAddress i2cAddress, uint16_t frequency) {
  _firstVpin = firstVpin;
    _nPins = (nPins > 16) ? 16 : nPins;
  _I2CAddress = i2cAddress;
  // Calculate prescaler value for PWM clock
  if (frequency > 1526) frequency = 1526;
  else if (frequency < 24) frequency = 24;
  prescaler = FREQUENCY_OSCILLATOR / 4096 / frequency;
  // To save RAM, space for servo configuration is not allocated unless a pin is used.
  // Initialise the pointers to NULL.
  for (int i=0; i<_nPins; i++)
    _servoData[i] = NULL;

  addDevice(this);

  // Initialise structure used for setting pulse rate
  requestBlock.setWriteParams(_I2CAddress, outputBuffer, sizeof(outputBuffer));
}

// Device-specific initialisation
void PCA9685::_begin() {
  I2CManager.begin();
  I2CManager.setClock(1000000); // Nominally able to run up to 1MHz on I2C
          // In reality, other devices including the Arduino will limit 
          // the clock speed to a lower rate.

  // Initialise I/O module here.
  if (I2CManager.exists(_I2CAddress)) {
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_SLEEP | MODE1_AI);    
    writeRegister(_I2CAddress, PCA9685_PRESCALE, prescaler);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_AI);
    writeRegister(_I2CAddress, PCA9685_MODE1, MODE1_RESTART | MODE1_AI);
    // In theory, we should wait 500us before sending any other commands to each device, to allow
    // the PWM oscillator to get running.  However, we don't do any specific wait, as there's 
    // plenty of other stuff to do before we will send a command.
  #if defined(DIAG_IO)
    _display();
  #endif
  } else
    _deviceState = DEVSTATE_FAILED;
}

// Device-specific write function, invoked from IODevice::write().  
// For this function, the configured profile is used.
void PCA9685::_write(VPIN vpin, int value) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 Write VPIN:%u Value:%d"), vpin, value);
  #endif
  int pin = vpin - _firstVpin;
  if (value) value = 1;

  struct ServoData *s = _servoData[pin];
  if (s != NULL) {
    // Use configured parameters
    _writeAnalogue(vpin, value ? s->activePosition : s->inactivePosition, s->profile, s->duration);
  }  else {
     /* simulate digital pin on PWM */
      _writeAnalogue(vpin, value ? 4095 : 0, Instant | NoPowerOff, 0);     
      }
}

// Device-specific writeAnalogue function, invoked from IODevice::writeAnalogue().
// Profile is as follows:
//  Bit 7:     0=Set PWM to 0% to power off servo motor when finished
//             1=Keep PWM pulses on (better when using PWM to drive an LED)
//  Bits 6-0:  0           Use specified duration (defaults to 0 deciseconds)
//             1 (Fast)    Move servo in 0.5 seconds
//             2 (Medium)  Move servo in 1.0 seconds
//             3 (Slow)    Move servo in 2.0 seconds
//             4 (Bounce)  Servo 'bounces' at extremes.
//            
void PCA9685::_writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 WriteAnalogue VPIN:%u Value:%d Profile:%d Duration:%d %S"), 
    vpin, value, profile, duration, _deviceState == DEVSTATE_FAILED?F("DEVSTATE_FAILED"):F(""));
  #endif
  if (_deviceState == DEVSTATE_FAILED) return;
  int pin = vpin - _firstVpin;
  if (value > 4095) value = 4095;
  else if (value < 0) value = 0;

  struct ServoData *s = _servoData[pin];
  if (s == NULL) {
    // Servo pin not configured, so configure now using defaults
    s = _servoData[pin] = (struct ServoData *) calloc(sizeof(struct ServoData), 1);
    if (s == NULL) return;  // Check for memory allocation failure
    s->activePosition = 4095;
    s->inactivePosition = 0;
    s->currentPosition = value;
    s->profile = Instant | NoPowerOff;  // Use instant profile (but not this time)
  }

  // Animated profile.  Initiate the appropriate action.
  s->currentProfile = profile;
  uint8_t profileValue = profile & ~NoPowerOff;  // Mask off 'don't-power-off' bit.
  s->numSteps = profileValue==Fast ? 10 :   // 0.5 seconds
                profileValue==Medium ? 20 : // 1.0 seconds
                profileValue==Slow ? 40 :   // 2.0 seconds
                profileValue==Bounce ? sizeof(_bounceProfile)-1 : // ~ 1.5 seconds
                duration * 2 + 1; // Convert from deciseconds (100ms) to refresh cycles (50ms)
  s->stepNumber = 0;
  s->toPosition = value;
  s->fromPosition = s->currentPosition;
}

// _read returns true if the device is currently in executing an animation, 
//  changing the output over a period of time.
int PCA9685::_read(VPIN vpin) {
  if (_deviceState == DEVSTATE_FAILED) return 0;
  int pin = vpin - _firstVpin;
  struct ServoData *s = _servoData[pin];
  if (s == NULL) 
    return false; // No structure means no animation!
  else
    return (s->stepNumber < s->numSteps);
}

void PCA9685::_loop(unsigned long currentMicros) {
  for (int pin=0; pin<_nPins; pin++) {
    updatePosition(pin);
  }
  delayUntil(currentMicros + refreshInterval * 1000UL);
}

// Private function to reposition servo
// TODO: Could calculate step number from elapsed time, to allow for erratic loop timing.
void PCA9685::updatePosition(uint8_t pin) {
  struct ServoData *s = _servoData[pin];
  
  if (s == NULL) return; // No pin configuration/state data

  if (s->numSteps == 0) return; // No animation in progress

  if (s->stepNumber == 0 && s->fromPosition == s->toPosition) {
    // Go straight to end of sequence, output final position.
    s->stepNumber = s->numSteps-1;
  }

  if (s->stepNumber < s->numSteps) {
    // Animation in progress, reposition servo
    s->stepNumber++;
    if ((s->currentProfile & ~NoPowerOff) == Bounce) {
      // Retrieve step positions from array in flash
      byte profileValue = GETFLASH(&_bounceProfile[s->stepNumber]);
      s->currentPosition = map(profileValue, 0, 100, s->fromPosition, s->toPosition);
    } else {
      // All other profiles - calculate step by linear interpolation between from and to positions.
      s->currentPosition = map(s->stepNumber, 0, s->numSteps, s->fromPosition, s->toPosition);
    }
    // Send servo command
    writeDevice(pin, s->currentPosition);
  } else if (s->stepNumber < s->numSteps + _catchupSteps) {
    // We've finished animation, wait a little to allow servo to catch up
    s->stepNumber++;
  } else if (s->stepNumber == s->numSteps + _catchupSteps 
            && s->currentPosition != 0) {
#ifdef IO_SWITCH_OFF_SERVO
    if ((s->currentProfile & NoPowerOff) == 0) {
      // Wait has finished, so switch off PWM to prevent annoying servo buzz
      writeDevice(pin, 0);
    }
#endif
    s->numSteps = 0;  // Done now.
  }
}

// writeDevice takes a pin in range 0 to _nPins-1 within the device, and a value
// between 0 and 4095 for the PWM mark-to-period ratio, with 4095 being 100%.
void PCA9685::writeDevice(uint8_t pin, int value) {
  #ifdef DIAG_IO
  DIAG(F("PCA9685 I2C:%s WriteDevice Pin:%d Value:%d"), _I2CAddress.toString(), pin, value);
  #endif
  // Wait for previous request to complete
  uint8_t status = requestBlock.wait();
  if (status != I2C_STATUS_OK) {
    _deviceState = DEVSTATE_FAILED;
    DIAG(F("PCA9685 I2C:%s failed %S"), _I2CAddress.toString(), I2CManager.getErrorMessage(status));
  } else {
    // Set up new request.
    outputBuffer[0] = PCA9685_FIRST_SERVO + 4 * pin;
    outputBuffer[1] = 0;
    outputBuffer[2] = (value == 4095 ? 0x10 : 0);  // 4095=full on
    outputBuffer[3] = value & 0xff;
    outputBuffer[4] = value >> 8;
    I2CManager.queueRequest(&requestBlock);
  }
}

// Display details of this device.
void PCA9685::_display() {
  DIAG(F("PCA9685 I2C:%s Configured on Vpins:%u-%u %S"), _I2CAddress.toString(), (int)_firstVpin, 
    (int)_firstVpin+_nPins-1, (_deviceState==DEVSTATE_FAILED) ? F("OFFLINE") : F(""));
}

// Internal helper function for this device
static void writeRegister(byte address, byte reg, byte value) {
  I2CManager.write(address, 2, reg, value);
}

// Profile for a bouncing signal or turnout
// The profile below is in the range 0-100% and should be combined with the desired limits
// of the servo set by _activePosition and _inactivePosition.  The profile is symmetrical here,
// i.e. the bounce is the same on the down action as on the up action.  First entry isn't used.
const uint8_t FLASH PCA9685::_bounceProfile[30] = 
    {0,2,3,7,13,33,50,83,100,83,75,70,65,60,60,65,74,84,100,83,75,70,70,72,75,80,87,92,97,100};
