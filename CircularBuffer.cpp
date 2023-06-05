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

#include "CircularBuffer.hpp"

CircularBuffer::CircularBuffer(int inSize)
{
	this->head = 0;
	this->tail = 0;
	this->full = false;
	this->peakCount = 0;
	this->buffer = NULL;
	this->size = inSize;
#ifdef ARDUINO_ARCH_ESP32
	this->xSemaphore = NULL;
#endif
}

void CircularBuffer::begin(bool inMultiThread)
{
#ifdef ARDUINO_ARCH_ESP32
	if (inMultiThread)
	{
		this->xSemaphore = xSemaphoreCreateMutex();
	}
#endif
	this->buffer = new byte[this->size];
	this->clear();
}

bool CircularBuffer::CheckIfBeginDone()
{
	if (this->buffer == NULL)
	{
		DIAG(F("Error : CircularBuffer missing begin !"));
		return false;
	}
	return true;
}

void CircularBuffer::end()
{
	START_SEMAPHORE()
	if (!this->CheckIfBeginDone()) return;

	delete[] this->buffer;
	this->buffer = NULL;	// to be sure to crash at any attemps to use it !
	END_SEMAPHORE()
}

void CircularBuffer::clear()
{
	if (!this->CheckIfBeginDone()) return;

	START_SEMAPHORE()
	memset(this->buffer, 0, this->size);

	this->full = false;
	this->head = this->tail = 0;
	this->peakCount = 0;
	END_SEMAPHORE()
}

bool CircularBuffer::PushByte(byte inpData)
{
	if (!this->CheckIfBeginDone()) return false;

	bool ok = true;
	START_SEMAPHORE()

	if (!this->full)
	{
		this->buffer[this->head] = inpData;
		this->head++;
		this->full = this->head == this->tail;

		if (this->full)
		{
			ok = false;
			this->peakCount = this->size + 1;	// maximum size !
		}
	}

	if (!ok)
	{
		DIAG(F("Error : the byte has been lost ! Buffer is full !"));
	}

	END_SEMAPHORE()

	this->GetCount();	// update peakCount...
	return ok;
}

bool CircularBuffer::PushBytes(byte* inpData, int inDataLength)
{
	if (!this->CheckIfBeginDone()) return false;

	bool ok = true;
	START_SEMAPHORE()
	if (!this->full)
	{
		for (int i = 0; i < inDataLength; i++)
		{
			this->buffer[this->head] = inpData[i];
			this->head = (this->head + 1) % this->size;
			this->full = this->head == this->tail;

			if (this->full)
			{
				ok = false;
				this->peakCount = this->size + (inDataLength - i);	// maximum size !
				break;
			}
		}
	}

	if (!ok)
	{
		DIAG(F("Error : bytes has been lost ! Buffer is full !"));
	}

	END_SEMAPHORE()
	this->GetCount();	// update peakCount...
	return ok;
}

byte CircularBuffer::GetByte()
{
	if (!this->CheckIfBeginDone()) return 0;

	byte value = 0;
	if (this->isEmpty())
		return 0;

	START_SEMAPHORE()
	value = this->buffer[this->tail];
	this->tail = (this->tail + 1) % this->size;
	this->full = false;
	END_SEMAPHORE()

	return value;
}

int16_t CircularBuffer::GetInt16()
{
	if (!this->CheckIfBeginDone()) return 0;

	if (this->isEmpty() || this->GetCount() < 2)
		return 0;

	byte value1 = this->GetByte();
	byte value2 = this->GetByte();

	return int16_t((unsigned char)(value2) << 8 | (unsigned char)(value1));
}

int32_t CircularBuffer::GetInt32()
{
	if (!this->CheckIfBeginDone()) return 0;

	if (this->isEmpty() || this->GetCount() < 4)
		return 0;

	byte value1 = this->GetByte();
	byte value2 = this->GetByte();
	byte value3 = this->GetByte();
	byte value4 = this->GetByte();

	return int32_t(value4 << 24 | value3 << 16 | value2 << 8 | value1);
}

bool CircularBuffer::GetBytes(byte* inpData, int inDataLength)
{
	if (!this->CheckIfBeginDone()) return false;

	if (this->GetCount() < inDataLength)
		return false;

	START_SEMAPHORE()
	for (int i = 0; i < inDataLength; i++)
	{
		inpData[i] = this->buffer[this->tail];
		this->tail = (this->tail + 1) % this->size;
	}
	this->full = false;
	END_SEMAPHORE()

	return true;
}

int16_t CircularBuffer::GetInt16(byte* pBuffer, int inPos)
{
	byte value1 = pBuffer[inPos];
	byte value2 = pBuffer[inPos+1];

	return int16_t((unsigned char)(value2) << 8 | (unsigned char)(value1));
}

int32_t CircularBuffer::GetInt32(byte* pBuffer, int inPos)
{
	byte value1 = pBuffer[inPos];
	byte value2 = pBuffer[inPos + 1];
	byte value3 = pBuffer[inPos + 2];
	byte value4 = pBuffer[inPos + 3];

	return int32_t(value4 << 24 | value3 << 16 | value2 << 8 | value1);
}

void CircularBuffer::GetBytes(byte* pBuffer, int inPos, byte* inpData, int inDataLength)
{
	memcpy(pBuffer + inPos, inpData, inDataLength);
}

int CircularBuffer::GetCount()
{
	if (!this->CheckIfBeginDone()) return 0;

	int usedSize = 0;

	usedSize = this->size;

	if (!this->full)
	{
		if (this->head >= this->tail)
		{
			usedSize = this->head - this->tail;
		}
		else
		{
			usedSize = this->size + this->head - this->tail;
		}
	}

	if (usedSize > this->peakCount)
		this->peakCount = usedSize;

	return usedSize;
}

#ifdef DCCPP_DEBUG_MODE
#ifdef VISUALSTUDIO
const char textNum[] = "123456789";
const char textChars[] = "ABCDEF";
const char textSymb[] = "/*-+&$!:;,";
void CircularBuffer::Test()
{
	CircularBuffer test(20);

	test.begin(false);

	Serial.println("After all initialized");
	test.printCircularBuffer();

	test.PushBytes((byte*)textNum, 9);
	Serial.println("After nums pushed");
	test.printCircularBuffer();

	byte ret = test.GetByte();
	Serial.print("ret '1' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '2': ");
	Serial.println((int)ret);
	test.printCircularBuffer();

	test.PushBytes((byte*)textChars, 6);
	Serial.println("After chars pushed");
	test.printCircularBuffer();

	ret = test.GetByte();
	Serial.print("ret '3' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '4': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '5': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '6': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '7': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '8': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '9': ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret 'A': ");
	Serial.println((int)ret);
	test.printCircularBuffer();

	test.PushBytes((byte*)textSymb, 10);
	Serial.println("After chars pushed");
	test.printCircularBuffer();

	ret = test.GetByte();
	Serial.print("ret 'B' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret 'C' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret 'D' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret 'E' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret 'F' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '/' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '*' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '-' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '+' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '&' : ");
	Serial.println((int)ret);
	ret = test.GetByte();
	Serial.print("ret '$' : ");
	Serial.println((int)ret);
	test.printCircularBuffer();

	Serial.print("Final size : ");
	Serial.println((int)test.GetCount());
	Serial.print("Max used : ");
	Serial.println((int)test.GetPeakCount());

	// add too much data in the buffer...
	test.PushBytes((byte*)textSymb, 10);
	Serial.println("After chars pushed");
	test.PushBytes((byte*)textNum, 9);
	Serial.println("After chars pushed");
	test.printCircularBuffer();

	Serial.print("Final size : ");
	Serial.println((int)test.GetCount());
	Serial.print("Max used : ");
	Serial.println((int)test.GetPeakCount());
}
#endif

void CircularBuffer::printCircularBuffer()
{
	if (!this->CheckIfBeginDone()) return;

	if (this->full)
		Serial.println("FULL !");
	for (int i = 0; i < this->size; i++)
	{
		if (i == this->tail)
			Serial.print("Tail ");
		if (i == this->head)
			Serial.print("Head ");
		if (i != this->tail && i != this->head)
			Serial.print("     ");
		Serial.println((int)this->buffer[i]);
	}
}
#endif
