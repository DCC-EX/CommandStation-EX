/*
 *  © 2022 Peter Cole. All rights reserved.
 *
 *  This is the example configuration file for the EX-IOExpander pin map file.
 * 
 *  It is highly recommended to copy this to "myPinMap.h" and modify to suit your specific
 *  requirements.
 * 
 *  NOTE: Modifications to this file will be overwritten by future software updates.
 */
#ifndef MYPINMAP_H
#define MYPINMAP_H

/////////////////////////////////////////////////////////////////////////////////////
//  Define the number of I/O pins to be configured on the EX-IOExpander device
// 
#define NUMBER_OF_DIGITAL_PINS 12
#define NUMBER_OF_ANALOGUE_PINS 4

/////////////////////////////////////////////////////////////////////////////////////
//  Define the pin map
// 
//  You must define the correct number of pins as per NUMBER_OF_PINS above
//
static uint8_t digitalPinMap[NUMBER_OF_DIGITAL_PINS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static uint8_t analoguePinMap[NUMBER_OF_ANALOGUE_PINS] = {A0, A1, A2, A3};

#endif