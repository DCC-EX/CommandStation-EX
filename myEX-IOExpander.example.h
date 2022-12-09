/*
 *  © 2022 Peter Cole. All rights reserved.
 *
 *  This is the example configuration file for a custom EX-IOExpander pin map file.
 * 
 *  It is highly recommended to copy this to "myEX-IOExpander.h" and modify to suit your specific
 *  requirements.
 * 
 *  If you are simply using a default definition for a defined microcontroller, then you don't
 *  need to use this file, and instead can use one of the existing definitions.
 *  
 *  Refer to https://dcc-ex.com for the full documentation.
 * 
 *  NOTE: Modifications to this file will be overwritten by future software updates.
 */
#ifndef MYEX_IOEXPANDER_H
#define MYEX_IOEXPANDER_H

#define MY_CUSTOM_NANO F("MY_NANO_PINMAP"), \
  new EXIODigitalPinMap(12,2,3,4,5,6,7,8,9,10,11,12,13) \
  new EXIOAnaloguePinMap(4,A0,A1,A2,A3)

#endif