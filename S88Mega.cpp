//Define from config.h
#ifdef S88_MEGA

#include "S88Mega.h"

void S88Mega::S88_Init(byte bus0len, byte bus1len, byte bus2len, byte bus3len)
{
	byte i;
	Timer5_Off();
	portreg = 0;
	ledcounter = 0;
	rm.bus[0] = bus0len;
	rm.bus[1] = bus1len;
	rm.bus[2] = bus2len;
	rm.bus[3] = bus3len;
	rm.buslen = 0;
	BUS[0] = A0; BUS[1] = A1; BUS[2] = A2; BUS[3] = A3;
	for (i = 0; i < 4; i++) {
		if (rm.bus[i] > S88_CHAIN_MAX) rm.bus[i] = S88_CHAIN_MAX;
		if ((rm.bus[i] > 0) && (rm.bus[i] < 8)) rm.bus[i] = 8;
		if (rm.bus[i] > rm.buslen) rm.buslen = rm.bus[i];
		pinMode(BUS[i], INPUT);
	}
	for (i = 0; i < rm.buslen; i++) {
		RmBytes[i] = 0x00;
	}
	S88_Case = S88_SET_LOAD;
	InIndex = 0;
	S88_PORTDIR = B11110000;    // bit 0 - 3 = input , bit 4 - 7 = output
	S88_PORTOUT = B10000000;    // LED aus, Reset aus, Load aus, Clock aus 
	DIAG(F("S88 Initalized"));	
	for (i = 0; i < 4; i++) {
		DIAG(F("%s%d%s%d"), "Bus length ", i, " = ", rm.bus[i]);
	}
	DIAG(F("%s%d"), "Bus length max = ", rm.buslen);
	Timer5_Init();
}


void S88Mega::Timer5_Init(void)
{
#ifdef S88_USE_TIMER
	noInterrupts();
	Timer5_Off();
	TIMER5_100us;	// compare 1599, Prescaler 1	100us
	//the following lines are alternative timers if your sensors need special timings
	//	TIMER5_25us;	// compare 399, prescaler 1		25us
	//	TIMER5_50us;	// compare 799, Prescaler 1		50us
	//	TIMER5_200us;	// compare 3199, Prescaler 1	200us
	//	TIMER5_400us;	// compare 6399, Prescaler 1	400us
	//	TIMER5_6400us;	// compare 15999, Prescaler 1	1ms
	TIMSK5 |= (1 << OCIE5A);
	DIAG(F("Timer5 initialized 100us for S88 scan"));
	//	Serial.println("Timer1 initialized 100us");
	interrupts();
#endif 
}
#ifdef S88_USE_TIMER

ISR(TIMER5_COMPA_vect)
{
	S88Mega* instance = S88Mega::getInstance();
	if (instance != NULL) {
		instance->cycle();
	}
}
#endif

void S88Mega::Timer5_Off(void)
{
#ifdef S88_USE_TIMER
	TCCR5A = 0;
	TCCR5B = 0;
	TCNT5 = 0;
	TIMSK5 = 0;
#endif
}

void S88Mega::S88_Set_RM(byte value)
{
	portreg &= B10001111;
	portreg |= value;
	S88_PORTOUT = portreg;
}

boolean S88Mega::is_rm(byte rmindex, byte busindex)
{
	if (!rm.bus[busindex]) return (false);
	if (rm.bus[busindex] < rmindex) return (false);
	return (true);
}

void S88Mega::S88_Read(void)
{
	byte indata = S88_PORTIN & 0x0F;
	for (byte i = 0; i < 4; i++)
	{
		if (is_rm(InIndex, i))
		{
			if (bitRead(indata, i) != bitRead(RmBytes[InIndex], i))
			{
				bitWrite(RmBytes[InIndex], i, bitRead(indata, i));
				bitSet(RmBytes[InIndex], i + 4);
			}
		}
	}
}
void S88Mega::S88_CheckChanges(Print* stream)
{
	boolean rd;
	char mystr[8];
	char numstr[5];
	word addr;
	mystr[0] = '<';
	char* P_char;
	for (byte i = 0; i < rm.buslen; i++)
	{
		for (byte x = 0; x < 4; x++)
		{
			if (is_rm(i, x))
			{
				if (bitRead(RmBytes[i], x + 4))
				{
					addr = i + (S88AdrBase * x) + S88AdrBase;
					bitClear(RmBytes[i], x + 4);
					rd = bitRead(RmBytes[i], x);
					if (stream != NULL)	StringFormatter::send(stream, F("<%c %d>"), rd ? 'Q' : 'q', addr);
					if (rd) mystr[1] = 'Q'; else mystr[1] = 'q';
					mystr[2] = 32;
					mystr[3] = 0;
					itoa(addr, numstr, DEC);
					strcat(mystr, numstr);
					strcat(mystr, ">");
					P_char = &mystr[0];
					while (*P_char)
					{
						sensor.write((uint8_t)*P_char);
						P_char++;
					}
				}
			}
		}
	}
}
void S88Mega::S88_Status(void)
{
	for (byte i = 0; i < rm.buslen; i++)
	{
		for (byte x = 0; x < 4; x++)
		{
			if (is_rm(i, x)) bitSet(RmBytes[i], x + 4);
		}
	}
}
inline S88NextLoopStep& operator++(S88NextLoopStep& currentValue, int)
{
	if (currentValue == S88_SET_AFTER_RESET) {
		currentValue = static_cast<S88NextLoopStep>(0);
		return currentValue;
	}
	const int i = static_cast<int>(currentValue);
	currentValue = static_cast<S88NextLoopStep>(i + 1);
	return currentValue;
}

void S88Mega::loop() {
	switch (eNextLoopStep)
	{
	case S88_SET_CLOCK:
		S88_CLOCK;
		S88_Case++;
		return;
	case S88_SET_CLEAR_CLOCK:
		S88_NOSIGNAL;
		S88_Read();
		InIndex++;
		if (InIndex >= rm.buslen)
		{
			S88_Case = S88_SET_LOAD;
			return;
		}
		S88_Case = S88_SET_CLOCK;
		return;
	case S88_SET_LOAD:
		ledcounter++;
		if (ledcounter > 200)
		{
			ledcounter = 0;
			S88_LED_TOGGLE;
		}
		InIndex = 0;
		S88_LOAD;
		S88_Case++;
		return;
	case S88_SET_LOAD_CLOCK:
		S88_LOAD_CLOCK;
		S88_Case++;
		return;
	case S88_SET_LOAD_NOCLOCK:
		S88_LOAD;
		S88_Case++;
		return;
	case S88_SET_AFTERLOAD:
		S88_NOSIGNAL;
		S88_Case++;
		return;
	case S88_SET_RESET:
		InIndex = 0;
		merk = true;
		S88_Read();
		InIndex++;
		S88_RESET;
		S88_Case++;
		return;
	case S88_SET_AFTER_RESET:
		S88_NOSIGNAL;
		S88_Case = S88_SET_CLOCK;
		return;
	default:
		S88_NOSIGNAL;
		S88_Case = S88_SET_LOAD;
		return;
	}
}


#endif