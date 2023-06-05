/*
 *  © 2023 Thierry Paris / Locoduino
 *  All rights reserved.
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

//-------------------------------------------------------------------
#ifndef __CircularBuffer_Hpp__
#define __CircularBuffer_Hpp__
//-------------------------------------------------------------------

#include <Arduino.h>
#include "DIAG.h"

/** This is a thread-safe buffer of bytes, binary or not. Bytes are pushed on the top (its head), and got from the bottom of the 
* buffer (its tail...).
*/
class CircularBuffer
{
private:
	byte *buffer;	// buffer itself
	int size;			// total size of this buffer
	int head;			// index of the first free byte in the buffer
	int tail;			// index of the next byte to get.
	bool full;		// if true, no more space !
	int peakCount;	// biggest occupancy size since the creation of the buffer.
#ifdef ARDUINO_ARCH_ESP32
	SemaphoreHandle_t xSemaphore; // semaphore d'exclusion mutuelle
#endif

public:
	/** Constructor.
	@param inSize	
	*/
	CircularBuffer(int inSize);

	/** Initialize the list.
	@param inMultiThread	If the buffer is used in multi-thread environnement, initialize the semaphore before calling this begin().
	*/
	void begin(bool inMultiThread = false);

	/** Close the usage of this buffer and free the allocated memory.
	*/
	void end();

	/** Remove the full content of the buffer, ready for the next push.
	*/
	void clear();

	/** Add one byte in the buffer.
	@param inpData	byte to add.
	@return true if the byte have been pushed. false if the byte are lost due to max size reached...
	*/
	bool PushByte(byte inpData);

	/** Add some bytes in the buffer.
	@param inpData	pointer to bytes to add.
	@param inDataLength	number of bytes to add.
	@return true if all bytes have been pushed. false if one or more bytes are lost due to max size reached...
	*/
	bool PushBytes(byte* inpData, int inDataLength);

	/** Get the next byte from the buffer.
	@return first available byte, or 0.
	*/
	byte GetByte();

	/** Get the next two bytes from the buffer, to form an integer.
	@return integer created from two available bytes, or 0.
	*/
	int16_t GetInt16();

	/** Get the next four bytes from the buffer, to form an integer.
	@return integer created from two available bytes, or 0.
	*/
	int32_t GetInt32();

	/** Get some bytes.
	@param inpData	buffer to fill.
	@param inDataLength	number of bytes to get.
	@return true if all bytes have been get. false if there were not enough bytes in th buffer.
	*/
	bool GetBytes(byte* inpData, int inDataLength);

	/** Get the next two bytes from the given buffer starting from the given position, to form an integer.
	@param pBuffer	buffer to scan.
	@param inPos	Position of the first useful byte.
	@return integer created from two available bytes, or 0.
	*/
	static int16_t GetInt16(byte* pBuffer, int inPos);

	/** Get the next four bytes from the given buffer starting from the given position, to form a 32 bits integer.
	@param pBuffer	buffer to scan.
	@param inPos	Position of the first useful byte.
	@return integer created from four available bytes, or 0.
	*/
	static int32_t GetInt32(byte* pBuffer, int inPos);

	/** Get some bytes from the given buffer starting from the given position.
	@param pBuffer	buffer to scan.
	@param inPos	Position of the first useful byte.
	@param inpData	buffer to fill.
	@param inDataLength	number of bytes to get.
	@return true if all bytes have been get. false if there were not enough bytes in th buffer.
	*/
	static void GetBytes(byte *pBuffer, int inPos, byte* inpData, int inDataLength);

	/** Count the number of bytes in the buffer.
	@return number of bytes in the buffer.
	*/
	int GetCount();

	/** Get the maximum size used by the buffer since the beggining of its usage.
	@return maximum number of bytes in the buffer.
	*/
	int GetPeakCount() { return this->peakCount; }

	/** Check if the buffer is empty or not.
	*/
	bool isEmpty() const
	{
		//if head and tail are equal, we are empty
		return (!this->full && (this->head == this->tail));
	}

	/** Check if the buffer is full or not.
	*/
	bool isFull() const
	{
		//If tail is ahead the head by 1, we are full
		return this->full;
	}

	/** Check if the begin has been called. If not, the buffer is not allocated and any usage will crash the system !
	*/
	bool CheckIfBeginDone();

#ifdef DCCPP_DEBUG_MODE
#ifdef VISUALSTUDIO
	/** Unit test function
	*/
	static void Test();
#endif

	/** Print the list of messages in the stack.
	@remark Only available if DCCPP_DEBUG_MODE is defined.
	*/
	void printCircularBuffer();
#endif
};

#ifdef ARDUINO_ARCH_ESP32
#define START_SEMAPHORE()	\
	{ \
		byte semaphoreTaken = this->xSemaphore == NULL?1:0; \
		if (this->xSemaphore != NULL) \
			if (xSemaphoreTake(this->xSemaphore, (TickType_t)100) == pdTRUE) \
				semaphoreTaken = 1; \
		if (semaphoreTaken == 1)

#define END_SEMAPHORE()	\
		xSemaphoreGive(this->xSemaphore); \
	}

#define ABORT_SEMAPHORE()	\
		xSemaphoreGive(this->xSemaphore);
#else
#define START_SEMAPHORE()
#define END_SEMAPHORE()
#define ABORT_SEMAPHORE()
#endif

#endif