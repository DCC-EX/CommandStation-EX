/*
 * © 2020 Gregor Baues, Luiz Henrique Cassettari. All rights reserved.
 *  
 * This is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * 
 * See the GNU General Public License for more details <https://www.gnu.org/licenses/>
 */

#ifndef _ARDUINO_UNIQUE_ID_H_
#define _ARDUINO_UNIQUE_ID_H_

#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
#include <avr/boot.h>
#ifndef SIGRD
#define SIGRD 5
#endif
#elif defined(ARDUINO_ARCH_ESP8266)
#elif defined(ARDUINO_ARCH_ESP32)
#elif defined(ARDUINO_ARCH_SAM)
#elif defined(ARDUINO_ARCH_SAMD)
#elif defined(ARDUINO_ARCH_STM32)
#else
#error "ArduinoUniqueID only works on AVR, SAM, SAMD, STM32 and ESP Architecture"
#endif

#if defined(ARDUINO_ARCH_AVR)

#if defined(__AVR_ATmega328PB__)
#define UniqueIDsize 10
#else
#define UniqueIDsize 9
#endif

#define UniqueIDbuffer UniqueIDsize

#elif defined(ARDUINO_ARCH_ESP8266)
#define UniqueIDsize 4
#define UniqueIDbuffer 8
#elif defined(ARDUINO_ARCH_ESP32)
#define UniqueIDsize 6
#define UniqueIDbuffer 8
#elif defined(ARDUINO_ARCH_SAM)
#define UniqueIDsize 16
#define UniqueIDbuffer 16
#elif defined(ARDUINO_ARCH_SAMD)
#define UniqueIDsize 16
#define UniqueIDbuffer 16
#elif defined(ARDUINO_ARCH_STM32)
#define UniqueIDsize 12
#define UniqueIDbuffer 12
#endif

#define UniqueID8 (_UniqueID.id + UniqueIDbuffer - 8)
#define UniqueID (_UniqueID.id + UniqueIDbuffer - UniqueIDsize)

#define UniqueIDdump(stream)                      \
	{                                             \
		stream.print("UniqueID: ");       \
		for (size_t i = 0; i < UniqueIDsize; i++) \
		{                                         \
			if (UniqueID[i] < 0x10)               \
				stream.print("0");                \
			stream.print(UniqueID[i], HEX);       \
			stream.print(" ");                    \
		}                                         \
		stream.println();                         \
	}

#define UniqueID8dump(stream)                \
	{                                        \
		stream.print("UniqueID: ");  \
		for (size_t i = 0; i < 8; i++)       \
		{                                    \
			if (UniqueID8[i] < 0x10)         \
				stream.print("0");           \
			stream.print(UniqueID8[i], HEX); \
			stream.print(" ");               \
		}                                    \
		stream.println();                    \
	}

class ArduinoUniqueID
{
  public:
	ArduinoUniqueID();
	uint8_t id[UniqueIDbuffer];
};

extern ArduinoUniqueID _UniqueID;

#endif