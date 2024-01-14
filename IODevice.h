/*
 *  © 2023, Paul Antoine, Discord user @ADUBOURG
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

#ifndef iodevice_h
#define iodevice_h

// Define symbol DIAG_IO to enable diagnostic output
//#define DIAG_IO Y

// Define symbol DIAG_LOOPTIMES to enable CS loop execution time to be reported
//#define DIAG_LOOPTIMES

// Define symbol IO_SWITCH_OFF_SERVO to set the PCA9685 output to 0 when an 
// animation has completed.  This switches off the servo motor, preventing 
// the continuous buzz sometimes found on servos, and reducing the 
// power consumption of the servo when inactive.
// It is recommended to enable this, unless it causes you problems.
#define IO_SWITCH_OFF_SERVO

#include "DIAG.h"
#include "FSH.h"
#include "I2CManager.h"
#include "inttypes.h"

typedef uint16_t VPIN;
// Limit VPIN number to max 32767.  Above this number, printing often gives negative values.
// This should be enough for 99% of users.
#define VPIN_MAX 32767  
#define VPIN_NONE 65535

/* 
 * Callback support for state change notification from an IODevice subclass to a 
 * handler, e.g. Sensor object handling.
 */

class IONotifyCallback {
public: 
  typedef void IONotifyCallbackFunction(VPIN vpin, int value);
  static void add(IONotifyCallbackFunction *function) {
    IONotifyCallback *blk = new IONotifyCallback(function);
    if (first) blk->next = first;
    first = blk;
  }
  static void invokeAll(VPIN vpin, int value) {
    for (IONotifyCallback *blk = first; blk != NULL; blk = blk->next)
      blk->invoke(vpin, value);
  }
  static bool hasCallback() {
    return first != NULL;
  }
private:
  IONotifyCallback(IONotifyCallbackFunction *function) { invoke = function; };
  IONotifyCallback *next = 0;
  IONotifyCallbackFunction *invoke = 0;
  static IONotifyCallback *first;
};

/*
 * IODevice class
 * 
 * This class is the basis of the Hardware Abstraction Layer (HAL) for
 * the DCC++EX Command Station.  All device classes derive from this.
 * 
 */

class IODevice {
public:

  // Parameter values to identify type of call to IODevice::configure.
  typedef enum : uint8_t {
    CONFIGURE_INPUT = 1,
    CONFIGURE_SERVO = 2,
    CONFIGURE_OUTPUT = 3,
    CONFIGURE_ANALOGOUTPUT = 4,
    CONFIGURE_ANALOGINPUT = 5,
  } ConfigTypeEnum;

  typedef enum : uint8_t {
    DEVSTATE_DORMANT = 0,
    DEVSTATE_PROBING = 1,
    DEVSTATE_INITIALISING = 2,
    DEVSTATE_NORMAL = 3,
    DEVSTATE_SCANNING = 4,
    DEVSTATE_FAILED = 5,
  } DeviceStateEnum;

  // Static functions to find the device and invoke its member functions

  // begin is invoked to create any standard IODevice subclass instances.
  // Also, the _begin method of any existing instances is called from here.
  static void begin();

  // reset function to invoke all driver's _begin() methods again, to
  // reset the state of the devices and reinitialise.
  static void reset();

  // configure is used invoke an IODevice instance's _configure method
  static bool configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]);

  // User-friendly function for configuring an input pin.
  inline static bool configureInput(VPIN vpin, bool pullupEnable) {
    int params[] = {pullupEnable};
    return IODevice::configure(vpin, CONFIGURE_INPUT, 1, params);
  }

  // User-friendly function for configuring a servo pin.
  inline static bool configureServo(VPIN vpin, uint16_t activePosition, uint16_t inactivePosition, uint8_t profile=0, uint16_t duration=0, uint8_t initialState=0) {
    int params[] = {(int)activePosition, (int)inactivePosition, profile, (int)duration, initialState};
    return IODevice::configure(vpin, CONFIGURE_SERVO, 5, params);
  }

  // write invokes the IODevice instance's _write method.
  static void write(VPIN vpin, int value);

  // write invokes the IODevice instance's _writeAnalogue method (not applicable for digital outputs)
  static void writeAnalogue(VPIN vpin, int value, uint8_t profile=0, uint16_t duration=0);

  // isBusy returns true if the device is currently in an animation of some sort, e.g. is changing
  //  the output over a period of time.
  static bool isBusy(VPIN vpin);

  // check whether the pin supports notification.  If so, then regular _read calls are not required.
  static bool hasCallback(VPIN vpin);

  // read invokes the IODevice instance's _read method.
  static int read(VPIN vpin);

  // read invokes the IODevice instance's _readAnalogue method.
  static int readAnalogue(VPIN vpin);
  static int configureAnalogIn(VPIN vpin);

  // loop invokes the IODevice instance's _loop method.
  static void loop();

  static void DumpAll();

  // exists checks whether there is a device owning the specified vpin
  static bool exists(VPIN vpin);

  // getStatus returns the state of the device at the specified vpin
  static uint8_t getStatus(VPIN vpin);

  // Enable shared interrupt on specified pin for GPIO extender modules.  The extender module
  // should pull down this pin when requesting a scan.  The pin may be shared by multiple modules.
  // Without the shared interrupt, input states are scanned periodically to detect changes on 
  // GPIO extender pins.  If a shared interrupt pin is configured, then input states are scanned
  // only when the shared interrupt pin is pulled low.  The external GPIO module releases the pin
  // once the GPIO port concerned has been read.
  void setGPIOInterruptPin(int16_t pinNumber);

  // Method to check if pins will overlap before creating new device. 
  static bool checkNoOverlap(VPIN firstPin, uint8_t nPins=1, 
                  I2CAddress i2cAddress=0, bool silent=false);

  // Method used by IODevice filters to locate slave pins that may be overlayed by their own
  // pin range.  
  IODevice *findDeviceFollowing(VPIN vpin);

  // Method to write new state (optionally implemented within device class)
  virtual void _write(VPIN vpin, int value) {
    (void)vpin; (void)value;
  };

  // Method to write an 'analogue' value (optionally implemented within device class)
  virtual void _writeAnalogue(VPIN vpin, int value, uint8_t param1=0, uint16_t param2=0) {
    (void)vpin; (void)value; (void) param1; (void)param2;
  };

  // Method to read digital pin state (optionally implemented within device class)
  virtual int _read(VPIN vpin) { 
    (void)vpin; 
    return 0;
  };

  // Method to read analogue pin state (optionally implemented within device class)
  virtual int _readAnalogue(VPIN vpin) { 
    (void)vpin; 
    return 0;
  };

protected:
  
  // Constructor
  IODevice(VPIN firstVpin=0, int nPins=0) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _nextEntryTime = 0;
    _I2CAddress=0;
  }

  // Method to perform initialisation of the device (optionally implemented within device class)
  virtual void _begin() {}

  // Method to check whether the vpin corresponds to this device
  bool owns(VPIN vpin);

  // Method to configure device (optionally implemented within device class)
  virtual bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) { 
    (void)vpin; (void)configType; (void)paramCount; (void)params; // Suppress compiler warning.
    return false;
  };

  virtual int _configureAnalogIn(VPIN vpin) { 
    (void)vpin; 
    return 0;
  };

  // Method to perform updates on an ongoing basis (optionally implemented within device class)
  virtual void _loop(unsigned long currentMicros) {
    delayUntil(currentMicros + 0x7fffffff); // Largest time in the future!  Effectively disable _loop calls.
  };

  // Method for displaying info on DIAG output (optionally implemented within device class)
  virtual void _display();

  // Destructor
  virtual ~IODevice() {};

  // Non-virtual function
  void delayUntil(unsigned long futureMicrosCount) {
    _nextEntryTime = futureMicrosCount;
  }
  
  // Common object fields.
  VPIN _firstVpin;
  int _nPins;
  I2CAddress _I2CAddress;
  // Flag whether the device supports callbacks.
  bool _hasCallback = false;

  // Pin number of interrupt pin for GPIO extender devices.  The extender module will pull this
  //  pin low if an input changes state.
  int16_t _gpioInterruptPin = -1;
    
  // Static support function for subclass creation
  static void addDevice(IODevice *newDevice, IODevice *slaveDevice = NULL);

  // Method to find device handling Vpin
  static IODevice *findDevice(VPIN vpin);

  // Current state of device
  DeviceStateEnum _deviceState = DEVSTATE_DORMANT;

private:
  IODevice *_nextDevice = 0;
  unsigned long _nextEntryTime;
  static IODevice *_firstDevice;

  static IODevice *_nextLoopDevice;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for PCA9685 16-channel PWM module.
 */
 
class PCA9685 : public IODevice {
public:
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress, uint16_t frequency = 50);
  enum ProfileType : uint8_t {
    Instant = 0,  // Moves immediately between positions (if duration not specified)
    UseDuration = 0, // Use specified duration
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4,   // For semaphores/turnouts with a bit of bounce!!
    NoPowerOff = 0x80, // Flag to be ORed in to suppress power off after move.
  };

private:
  // Constructor
  PCA9685(VPIN vpin, int nPins, I2CAddress i2cAddress, uint16_t frequency);
  // Device-specific initialisation
  void _begin() override;
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override;
  // Device-specific write functions.
  void _write(VPIN vpin, int value) override;
  void _writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration) override;
  int _read(VPIN vpin) override; // returns the digital state or busy status of the device
  void _loop(unsigned long currentMicros) override;
  void updatePosition(uint8_t pin);
  void writeDevice(uint8_t pin, int value);
  void _display() override;
  

  struct ServoData {
    uint16_t activePosition : 12; // Config parameter
    uint16_t inactivePosition : 12; // Config parameter
    uint16_t currentPosition : 12;
    uint16_t fromPosition : 12;
    uint16_t toPosition : 12; 
    uint8_t profile;  // Config parameter
    uint16_t stepNumber; // Index of current step (starting from 0)
    uint16_t numSteps;  // Number of steps in animation, or 0 if none in progress.
    uint8_t currentProfile; // profile being used for current animation.
    uint16_t duration; // time (tenths of a second) for animation to complete.
  }; // 14 bytes per element, i.e. per pin in use
  
  struct ServoData *_servoData [16];

  static const uint8_t _catchupSteps = 5; // number of steps to wait before switching servo off
  static const uint8_t FLASH _bounceProfile[30];

  const unsigned int refreshInterval = 50; // refresh every 50ms

  // structures for setting up non-blocking writes to servo controller
  I2CRB requestBlock;
  uint8_t outputBuffer[5];
  uint8_t prescaler; // clock prescaler for setting PWM frequency
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for DCC accessory decoder.
 */
 
class DCCAccessoryDecoder: public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, int DCCAddress, int DCCSubaddress);

private:
  // Constructor
  DCCAccessoryDecoder(VPIN firstVpin, int nPins, int DCCAddress, int DCCSubaddress);
  // Device-specific write function.
  void _begin() override;
  void _write(VPIN vpin, int value) override;
  void _display() override;
  int _packedAddress;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 *  IODevice subclass for arduino input/output pins.
 */
 
class ArduinoPins: public IODevice {
public:
  static void create(VPIN firstVpin, int nPins) {
    addDevice(new ArduinoPins(firstVpin, nPins));
  }
  
  static void fastWriteDigital(uint8_t pin, uint8_t value);
  static bool fastReadDigital(uint8_t pin);

private:
  // Constructor
  ArduinoPins(VPIN firstVpin, int nPins);

  // Device-specific pin configuration
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override;
  // Device-specific write function.
  void _write(VPIN vpin, int value) override;
  // Device-specific read functions.
  int _read(VPIN vpin) override;
  int _readAnalogue(VPIN vpin) override;
  int _configureAnalogIn(VPIN vpin) override;
  void _display() override;


  uint8_t *_pinPullups;
  uint8_t *_pinModes; // each bit is 1 for output, 0 for input
  uint8_t *_pinInUse; 
};

#ifndef IO_NO_HAL
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-Turntable.
 */
 
class EXTurntable : public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, I2CAddress I2CAddress);
  // Constructor
  EXTurntable(VPIN firstVpin, int nPins, I2CAddress I2CAddress);
  enum ActivityNumber : uint8_t {
    Turn = 0,             // Rotate turntable, maintain phase
    Turn_PInvert = 1,     // Rotate turntable, invert phase
    Home = 2,             // Initiate homing
    Calibrate = 3,        // Initiate calibration sequence
    LED_On = 4,           // Turn LED on
    LED_Slow = 5,         // Set LED to a slow blink
    LED_Fast = 6,         // Set LED to a fast blink
    LED_Off = 7,          // Turn LED off
    Acc_On = 8,           // Turn accessory pin on
    Acc_Off = 9,          // Turn accessory pin off
  };

private:
  // Device-specific write function.
  void _begin() override;
  void _loop(unsigned long currentMicros) override;
  int _read(VPIN vpin) override;
  void _broadcastStatus (VPIN vpin, uint8_t status, uint8_t activity);
  void _writeAnalogue(VPIN vpin, int value, uint8_t activity, uint16_t duration) override;
  void _display() override;
  uint8_t _stepperStatus;
  uint8_t _previousStatus;
  uint8_t _currentActivity;
};
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////


// IODevice framework for invoking user-written functions.
// To use, define a function that you want to be regularly
// invoked, and then create an instance of UserAddin.  
// For example, you can show the status, on screen 3, of the first eight
// locos in the speed table:
// 
// void updateLocoScreen() {
//   for (int i=0; i<8; i++) {
//     if (DCC::speedTable[i].loco > 0) {
//       int speed = DCC::speedTable[i].speedCode;
//       SCREEN(3, i, F("Loco:%4d %3d %c"), DCC::speedTable[i].loco,
//         speed & 0x7f, speed & 0x80 ? 'R' : 'F');
//     }
//   }
// }
//
// void halSetup() {
//   ...
//   UserAddin(updateLocoScreen, 1000);  // Update every 1000ms
//   ...
// }
//
class UserAddin : public IODevice {
private:
  void (*_invokeUserFunction)();
  int _delay; // milliseconds
public:
  UserAddin(void (*func)(), int delay) {
    _invokeUserFunction = func; 
    _delay = delay; 
    addDevice(this); 
  }
  // userFunction has no return value, no parameter.  delay is in milliseconds.
  static void create(void (*userFunction)(), int delay) {
    new UserAddin(userFunction, delay);
  }
protected:
  void _begin() { _display(); }
  void _loop(unsigned long currentMicros) override {
    _invokeUserFunction();
    // _loop won't be called again until _delay ms have elapsed.
    delayUntil(currentMicros + _delay * 1000UL);
  }
  void _display() override {
    DIAG(F("UserAddin run every %dms"), _delay);
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// This HAL device driver is intended for communication in automation
// sequences.  A VPIN can be SET or RESET within a sequence, and its
// current state checked elsewhere using IF, IFNOT, AT etc. or monitored
// from JMRI using a Sensor object (DCC-EX <S ...> command).
// Alternatively, the flag can be set from JMRI and other interfaces
// using the <Z ...> command, to enable or disable actions within a sequence.
// 
// Example of configuration in halSetup.h:
// 
//  FLAGS::create(32000, 128);
// 
// or in myAutomation.h:
//  
//  HAL(FLAGS, 32000, 128);
// 
// Both create 128 flags numbered with VPINs 32000-32127.
// 
//

class FLAGS : IODevice {
private:
  uint8_t *_states = NULL;

public:
  static void create(VPIN firstVpin, unsigned int nPins) {
    if (checkNoOverlap(firstVpin, nPins))
        new FLAGS(firstVpin, nPins);
  }

protected:
  // Constructor performs static initialisation of the device object
  FLAGS (VPIN firstVpin, int nPins) {
    _firstVpin = firstVpin;
    _nPins = nPins;
    _states = (uint8_t *)calloc(1, (_nPins+7)/8);
    if (!_states) {
      DIAG(F("FLAGS: ERROR Memory Allocation Failure"));
      return;
    }

    addDevice(this);
  }

  int _read(VPIN vpin) override {
    int pin = vpin - _firstVpin;
    if (pin >= _nPins || pin < 0) return 0;
    uint8_t mask = 1 << (pin & 7);
    return (_states[pin>>3] & mask) ? 1 : 0;
  }

  void _write(VPIN vpin, int value) override {
    int pin = vpin - _firstVpin;
    if (pin >= _nPins || pin < 0) return;
    uint8_t mask = 1 << (pin & 7);
    if (value) 
      _states[pin>>3] |= mask;
    else
      _states[pin>>3] &= ~mask;
  }

  void _display() override {
    DIAG(F("FLAGS configured on VPINs %u-%u"),
      _firstVpin, _firstVpin+_nPins-1);
  }

};

#include "IO_MCP23008.h"
#include "IO_MCP23017.h"
#include "IO_PCF8574.h"
#include "IO_PCF8575.h"
#include "IO_PCA9555.h"
#include "IO_duinoNodes.h"
#include "IO_EXIOExpander.h"
#include "IO_trainbrains.h"


#endif // iodevice_h
