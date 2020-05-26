#include "Railcom.h"
#include "Hardware.h"

    void Railcom::startCutout() {
      return;
      /*** todo when ready ***
      interruptState=0;
      Hardware::setSingleCallback(RAILCOM_T0,interrupt);
      *************/
      }
      
    void Railcom::interrupt() {
      /*** TODO when ready .. railcom state machine
      switch (interruptState) {
        case 0: //  
              Hardware::setPower(true,false);
              state=1;
              nextInterrupt=RAILCOM_T1;
              break;
      }
      **********************/
      }
    byte Railcom::interruptState;
    byte Railcom::bitsReceived;
    byte Railcom::buffer[MAX_BUFFER];    
