Using Lew's Duino Gear boards:

1. DNIN8 Input
   This is a shift-register implementation of a digital input collector. 
   Multiple DNIN8 may be connected in sequence but it is IMPORTANT that the software 
   configuratuion correctly represents the number of boards connected otherwise the results will be meaningless. 

   Use in myAnimation.h

   HAL(IO_DNIN8, firstVpin, numPins, clockPin, latchPin, dataPin)

   e.g. 
   HAL(IO_DNIN8, 400, 16, 40, 42, 44)
   This will create virtaul pins 400-415 using two DNIN8 boards connected in sequence. 
   Vpins 400-407 will be on the first board (closest to the CS) and 408-415 on the second.

   Note: 16 pins uses two boards. You may specify a non-multiple-of-8 pins but this will be rounded up to a multiple of 8 and you must connect ONLY the number of boards that this takes. 
      
   This example uses Arduino GPIO pins 40,42,44 as these are conveniently side-by-side on a Mega which is easier when you are using a 3 strand cable. 
 