#include <Arduino.h>
#include <CommandStation.h>
#include <ArduinoTimers.h>

const uint8_t kIRQmicros = 29;
const uint8_t kNumLocos = 50;

////////////////////////////////////////////////////////////////
// Motor driver selection:
// Comment out all but the two lines that you want to use

// DCCMain* mainTrack = DCCMain::Create_WSM_SAMCommandStation_Main(kNumLocos);
// DCCService* progTrack = DCCService::Create_WSM_SAMCommandStation_Prog();

// DCCMain* mainTrack = DCCMain::Create_Arduino_L298Shield_Main(kNumLocos);
// DCCService* progTrack = DCCService::Create_Arduino_L298Shield_Prog();
  
DCCMain* mainTrack = DCCMain::Create_Pololu_MC33926Shield_Main(kNumLocos);
DCCService* progTrack = DCCService::Create_Pololu_MC33926Shield_Prog();

////////////////////////////////////////////////////////////////

void waveform_IrqHandler() {
  mainTrack->interruptHandler();
  progTrack->interruptHandler();
}

#if defined(ARDUINO_ARCH_SAMD)
void SERCOM4_Handler()
{   
  mainTrack->railcom.getSerial()->IrqHandler();
}
#endif

void setup() {
  mainTrack->setup();
  progTrack->setup();

  // TimerA is TCC0 on SAMD21, Timer1 on MEGA2560, and Timer1 on MEGA328
  // We will fire an interrupt every 29us to generate the signal on the track 
  TimerA.initialize();
  TimerA.setPeriod(kIRQmicros);
  TimerA.attachInterrupt(waveform_IrqHandler);
  TimerA.start();

#if defined (ARDUINO_ARCH_SAMD)
  CommManager::registerInterface(new USBInterface(SerialUSB));
  Wire.begin();       // Needed for EEPROM to work
#elif defined(ARDUINO_ARCH_AVR)
  CommManager::registerInterface(new SerialInterface(Serial));
#endif

  EEStore::init();

  // Set up the string parser to accept commands from the interfaces
  DCCEXParser::init(mainTrack, progTrack);       

  CommManager::showInitInfo();           
}

void loop() {
  CommManager::update();
  mainTrack->loop();
  progTrack->loop();
}

