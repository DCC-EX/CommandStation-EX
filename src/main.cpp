#include <Arduino.h>
#include <CommandStation.h>
#include <ArduinoTimers.h>

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
    CommManager::registerInterface(new USBInterface(SerialUSB));     // Register SerialUSB as an interface
    
    mainTrack->int_timer->initialize();
    mainTrack->int_timer->setPeriod(58);
    mainTrack->int_timer->attachInterrupt(main_IrqHandler);
    mainTrack->int_timer->start();
    
    progTrack->int_timer->initialize();
    progTrack->int_timer->setPeriod(58);
    progTrack->int_timer->attachInterrupt(prog_IrqHandler);
    progTrack->int_timer->start();
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