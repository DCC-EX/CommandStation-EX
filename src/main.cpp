#include <Arduino.h>
#include <CommandStation.h>

DCC* mainTrack = DCC::Create_WSM_SAMCommandStation_Main(50);
DCC* progTrack = DCC::Create_WSM_SAMCommandStation_Prog(2);

void main_IrqHandler() {
    mainTrack->interrupt_handler();
}

void prog_IrqHandler() {
    progTrack->interrupt_handler();
}

void setup() {
#if defined (ATSAMD21G)
    CommManager::registerInterface(new SerialInterface(SerialUSB));     // Register SerialUSB as an interface
    
    TimerTCC0.initialize();
    TimerTCC0.setPeriod(58);
    TimerTCC0.attachInterrupt(main_IrqHandler);
    TimerTCC0.start();
    
    TimerTCC1.initialize();
    TimerTCC1.setPeriod(58);
    TimerTCC1.attachInterrupt(prog_IrqHandler);
    TimerTCC1.start();
#elif defined(ATMEGA2560)
    CommManager::registerInterface(new SerialInterface(Serial));        // Register Serial (USB port on mega/uno) as an interface

    Timer1.initialize();
    Timer1.setPeriod(58);
    Timer1.attachInterrupt(main_IrqHandler);
    Timer1.start();
    
    Timer3.initialize();
    Timer3.setPeriod(58);
    Timer3.attachInterrupt(prog_IrqHandler);
    Timer3.start();
#endif

	StringParser::init(mainTrack, progTrack);       // Set up the string parser to accept commands from the interfaces
	CommManager::showInitInfo();                
}

void loop() {
    CommManager::update();
	mainTrack->check();
	progTrack->check();
}