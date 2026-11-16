#ifndef IO_EXTURNTABLE_H
#define IO_EXTURNTABLE_H
#include "IODevice.h"

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

  
class EXTurntable : public IODevice {
public:
  static void create(VPIN firstVpin, int nPins, I2CAddress I2CAddress);
  // Constructor
  EXTurntable(VPIN firstVpin, int nPins, I2CAddress I2CAddress);
 
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
#endif // IO_EXTURNTABLE_H
