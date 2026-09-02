/*
 *  © 2022 Paul M. Antoine
 *  © 2021 Neil McKechnie
 *  © 2021 Harald Barth
 *  © 2021 Fred Decker
 *  All rights reserved.
 *  
 *  This file is part of CommandStation-EX
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
#ifndef FSH_h
#define FSH_h

/* This is an architecture support file to manage the differences 
 *  between the nano/uno.mega and the later nanoEvery, unoWifiRev2 etc
 *  
 *  IMPORTANT:
 *  To maintain portability the main code should NOT contain ANY references 
 *  to the following: 
 *  
 *  __FlashStringHelper     Use FSH instead.
 *  PROGMEM                 use FLASH instead
 *  pgm_read_byte_near      use GETFLASH instead.
 *  pgm_read_word_near      use GETFLASHW instead.
 * 
 *  Also:
 *    HIGHFLASH    -  PROGMEM forced to end of link so needs far pointers.
 *    GETHIGHFLASH,GETHIGHFLASHW to access them
 *  
 */
#include <Arduino.h>
// Non-AVR Flat-memory devices have no need of this support so can be remapped to normal memory access
#ifdef F
  #undef F
#endif
#ifdef FLASH
  #undef FLASH
#endif
#define F(str) (str)
typedef char FSH; 
#define FLASH
#define HIGHFLASH
#define HIGHFLASH3
#define GETFARPTR(data) ((uint32_t)(data))
#define GETFLASH(addr) (*(const byte *)(addr))
#define GETHIGHFLASH(data,offset)  (*(const byte *)(GETFARPTR(data)+offset))
#define GETHIGHFLASHW(data,offset) (*(const uint16_t *)(GETFARPTR(data)+offset))
#define COPYHIGHFLASH(target,base,offset,length) \
  memcpy(target,(byte *)&base + offset,length)  
#define STRCPY_P strcpy
#define STRCMP_P strcmp
#define STRNCPY_P strncpy
#define STRNCMP_P strncmp
#define STRLEN_P strlen
#define STRCHR_P strchr
#endif
