/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ST7735.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>


// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ST7735::Adafruit_ST7735(uint8_t cs, uint8_t rs, uint8_t sid,
 uint8_t sclk, uint8_t rst) : Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT)
{
	_cs   = cs;
	_rs   = rs;
	_sid  = sid;
	_sclk = sclk;
	_rst  = rst;
	hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ST7735::Adafruit_ST7735(uint8_t cs, uint8_t rs, uint8_t rst) :
  Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT) {
	_cs   = cs;
	_rs   = rs;
	_rst  = rst;
	hwSPI = true;
	_sid  = _sclk = (uint8_t)-1;
}


/***************************************************************/
/*     Arduino Uno, Leonardo, Mega, Teensy 2.0, etc            */
/***************************************************************/
#if defined(__AVR__ )
inline void Adafruit_ST7735::writebegin()
{
}

inline void Adafruit_ST7735::spiwrite(uint8_t c)
{
	if (hwSPI) {
		SPDR = c;
		while(!(SPSR & _BV(SPIF)));
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void Adafruit_ST7735::writecommand(uint8_t c)
{
	*rsport &= ~rspinmask;
	*csport &= ~cspinmask;
	spiwrite(c);
	*csport |= cspinmask;
}

void Adafruit_ST7735::writedata(uint8_t c)
{
	*rsport |=  rspinmask;
	*csport &= ~cspinmask;
	spiwrite(c);
	*csport |= cspinmask;
} 

void Adafruit_ST7735::writedata16(uint16_t d)
{
	*rsport |=  rspinmask;
	*csport &= ~cspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
	*csport |= cspinmask;
} 

void Adafruit_ST7735::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}

/***************************************************************/
/*     Arduino Due                                             */
/***************************************************************/
#elif defined(__SAM3X8E__)
inline void Adafruit_ST7735::writebegin()
{
}

inline void Adafruit_ST7735::spiwrite(uint8_t c)
{
	//Serial.println(c, HEX);
	if (hwSPI) {
		SPI.transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) dataport->PIO_SODR |= datapinmask;
			else        dataport->PIO_CODR |= datapinmask;
			clkport->PIO_SODR |= clkpinmask;
			clkport->PIO_CODR |= clkpinmask;
		}
	}
}

void Adafruit_ST7735::writecommand(uint8_t c)
{
	rsport->PIO_CODR |=  rspinmask;
	csport->PIO_CODR  |=  cspinmask;
	spiwrite(c);
	csport->PIO_SODR  |=  cspinmask;
}

void Adafruit_ST7735::writedata(uint8_t c)
{
	rsport->PIO_SODR |=  rspinmask;
	csport->PIO_CODR  |=  cspinmask;
	spiwrite(c);
	csport->PIO_SODR  |=  cspinmask;
} 

void Adafruit_ST7735::writedata16(uint16_t d)
{
	rsport->PIO_SODR |=  rspinmask;
	csport->PIO_CODR  |=  cspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
	csport->PIO_SODR  |=  cspinmask;
}

void Adafruit_ST7735::setBitrate(uint32_t n)
{
	uint32_t divider=1;
	while (divider < 255) {
		if (n >= 84000000 / divider) break;
		divider = divider - 1;
	}
	SPI.setClockDivider(divider);
}


/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline void Adafruit_ST7735::writebegin()
{
}

inline void Adafruit_ST7735::spiwrite(uint8_t c)
{
	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

void Adafruit_ST7735::writecommand(uint8_t c)
{
	if (hwSPI) {
		KINETISK_SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	} else {
		*rspin = 0;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void Adafruit_ST7735::writedata(uint8_t c)
{
	if (hwSPI) {
		KINETISK_SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void Adafruit_ST7735::writedata16(uint16_t d)
{
	if (hwSPI) {
		KINETISK_SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(d >> 8);
		spiwrite(d);
		*cspin = 1;
	}
}

static bool spi_pin_is_cs(uint8_t pin)
{
	if (pin == 2 || pin == 6 || pin == 9) return true;
	if (pin == 10 || pin == 15) return true;
	if (pin >= 20 && pin <= 23) return true;
	return false;
}

static uint8_t spi_configure_cs_pin(uint8_t pin)
{
        switch (pin) {
                case 10: CORE_PIN10_CONFIG = PORT_PCR_MUX(2); return 0x01; // PTC4
                case 2:  CORE_PIN2_CONFIG  = PORT_PCR_MUX(2); return 0x01; // PTD0
                case 9:  CORE_PIN9_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTC3
                case 6:  CORE_PIN6_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTD4
                case 20: CORE_PIN20_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTD5
                case 23: CORE_PIN23_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTC2
                case 21: CORE_PIN21_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTD6
                case 22: CORE_PIN22_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTC1
                case 15: CORE_PIN15_CONFIG = PORT_PCR_MUX(2); return 0x10; // PTC0
        }
        return 0;
}

#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))

void Adafruit_ST7735::setBitrate(uint32_t n)
{
	if (n >= 24000000) {
		ctar = CTAR_24MHz;
	} else if (n >= 16000000) {
		ctar = CTAR_16MHz;
	} else if (n >= 12000000) {
		ctar = CTAR_12MHz;
	} else if (n >= 8000000) {
		ctar = CTAR_8MHz;
	} else if (n >= 6000000) {
		ctar = CTAR_6MHz;
	} else {
		ctar = CTAR_4MHz;
	}
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	KINETISK_SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
	KINETISK_SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	KINETISK_SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	KINETISK_SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}

/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)
inline void Adafruit_ST7735::writebegin()
{
}

inline void Adafruit_ST7735::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (hwSPI) {
		SPI.transfer(c);
	} else if (hwSPI1) {
		SPI1.transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void Adafruit_ST7735::writecommand(uint8_t c)
{
	*rsport &= ~rspinmask;
	*csport &= ~cspinmask;
	spiwrite(c);
	*csport |= cspinmask;
}

void Adafruit_ST7735::writedata(uint8_t c)
{
	*rsport |=  rspinmask;
	*csport &= ~cspinmask;
	spiwrite(c);
	*csport |= cspinmask;
} 

void Adafruit_ST7735::writedata16(uint16_t d)
{
	*rsport |=  rspinmask;
	*csport &= ~cspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
	*csport |= cspinmask;
} 

void Adafruit_ST7735::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ST7735::commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	writebegin();
	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	while(numCommands--) {				// For each command...
		writecommand(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs--) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
		}

		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			delay(ms);
		}
	}
}


// Initialization code common to both 'B' and 'R' type displays
void Adafruit_ST7735::commonInit(const uint8_t *cmdList)
{
	colstart  = rowstart = 0; // May be overridden in init func

#ifdef __AVR__
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = portOutputRegister(digitalPinToPort(_cs));
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);

	if(hwSPI) { // Using hardware SPI
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE0);
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;


#elif defined(__SAM3X8E__)
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = digitalPinToPort(_cs);
	rsport    = digitalPinToPort(_rs);
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);

	if(hwSPI) { // Using hardware SPI
		SPI.begin();
		SPI.setClockDivider(21); // 4 MHz
		//Due defaults to 4mHz (clock divider setting of 21), but we'll set it anyway 
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE0);
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = digitalPinToPort(_sclk);
		dataport    = digitalPinToPort(_sid);
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		clkport ->PIO_CODR  |=  clkpinmask; // Set control bits to LOW (idle)
		dataport->PIO_CODR  |=  datapinmask; // Signals are ACTIVE HIGH
	}
	// toggle RST low to reset; CS low so it'll listen to us
	csport ->PIO_CODR  |=  cspinmask; // Set control bits to LOW (idle)


#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if ( spi_pin_is_cs(_cs) && spi_pin_is_cs(_rs)
	 && (_sid == 7 || _sid == 11)
	 && (_sclk == 13 || _sclk == 14)
	 && !(_cs ==  2 && _rs == 10) && !(_rs ==  2 && _cs == 10)
	 && !(_cs ==  6 && _rs ==  9) && !(_rs ==  6 && _cs ==  9)
	 && !(_cs == 20 && _rs == 23) && !(_rs == 20 && _cs == 23)
	 && !(_cs == 21 && _rs == 22) && !(_rs == 21 && _cs == 22) ) {
		hwSPI = true;
		if (_sclk == 13) {
			CORE_PIN13_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;
			SPCR.setSCK(13);
		} else {
			CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
			SPCR.setSCK(14);
		}
		if (_sid == 11) {
			CORE_PIN11_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(11);
		} else {
			CORE_PIN7_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(7);
		}
		ctar = CTAR_12MHz;
		pcs_data = spi_configure_cs_pin(_cs);
		pcs_command = pcs_data | spi_configure_cs_pin(_rs);
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
		KINETISK_SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
		KINETISK_SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
		KINETISK_SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
		KINETISK_SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	} else {
		hwSPI = false;
		cspin = portOutputRegister(digitalPinToPort(_cs));
		rspin = portOutputRegister(digitalPinToPort(_rs));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_sid));
		*cspin = 1;
		*rspin = 0;
		*clkpin = 0;
		*datapin = 0;
		pinMode(_cs, OUTPUT);
		pinMode(_rs, OUTPUT);
		pinMode(_sclk, OUTPUT);
		pinMode(_sid, OUTPUT);
	}
    // Teensy LC
#elif defined(__MKL26Z64__)
	hwSPI1 = false;
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	
	// See if pins are on standard SPI0
	if ((_sid == 7 || _sid == 11) && (_sclk == 13 || _sclk == 14)) {
		hwSPI = true;
	} else {
		hwSPI = false;
		if ((_sid == 0 || _sid == 21) && (_sclk == 20 )) {
			hwSPI1 = true;
		}
	}
 
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = portOutputRegister(digitalPinToPort(_cs));
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);

	if(hwSPI) { // Using hardware SPI
		if (_sclk == 14) SPI.setSCK(14);
		if (_sid == 7) SPI.setMOSI(7);
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE0);
	} else if(hwSPI1) { // Using hardware SPI
		SPI1.setSCK(_sclk);
		SPI1.setMOSI(_sid);
		SPI1.begin();
		SPI1.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI1.setBitOrder(MSBFIRST);
		SPI1.setDataMode(SPI_MODE0);
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;

#endif

	if (_rst) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(500);
		digitalWrite(_rst, LOW);
		delay(500);
		digitalWrite(_rst, HIGH);
		delay(500);
	}

	if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void Adafruit_ST7735::initB(void)
{
	commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void Adafruit_ST7735::initR(uint8_t options)
{
	commonInit(Rcmd1);
	if (options == INITR_GREENTAB) {
		commandList(Rcmd2green);
		colstart = 2;
		rowstart = 1;
	} else {
		// colstart, rowstart left at default '0' values
		commandList(Rcmd2red);
	}
	commandList(Rcmd3);

	// if black, change MADCTL color filter
	if (options == INITR_BLACKTAB) {
		writecommand(ST7735_MADCTL);
		writedata(0xC0);
	}

	tabcolor = options;
}


void Adafruit_ST7735::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	writecommand(ST7735_CASET); // Column addr set
	writedata16(x0+colstart);   // XSTART 
	writedata16(x1+colstart);   // XEND
	writecommand(ST7735_RASET); // Row addr set
	writedata16(y0+rowstart);   // YSTART
	writedata16(y1+rowstart);   // YEND
	writecommand(ST7735_RAMWR); // write to RAM
}


void Adafruit_ST7735::pushColor(uint16_t color)
{
	writebegin();
	writedata16(color);
}

void Adafruit_ST7735::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	setAddrWindow(x,y,x+1,y+1);
	writedata16(color);
}


void Adafruit_ST7735::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height) h = _height-y;
	setAddrWindow(x, y, x, y+h-1);
	while (h--) {
		writedata16(color);
	}
}


void Adafruit_ST7735::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(x, y, x+w-1, y);
	while (w--) {
		writedata16(color);
	}
}



void Adafruit_ST7735::fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void Adafruit_ST7735::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height)) return;
	if ((x + w - 1) >= _width)  w = _width  - x;
	if ((y + h - 1) >= _height) h = _height - y;
	setAddrWindow(x, y, x+w-1, y+h-1);
	for (y=h; y>0; y--) {
		for(x=w; x>0; x--) {
			writedata16(color);
		}
	}
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ST7735::setRotation(uint8_t m)
{
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = ST7735_TFTHEIGHT;
		break;
	case 1:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		}
		_width  = ST7735_TFTHEIGHT;
		_height = ST7735_TFTWIDTH;
		break;
	case 2:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_RGB);
		} else {
			writedata(MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = ST7735_TFTHEIGHT;
		break;
	case 3:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		_width  = ST7735_TFTHEIGHT;
		_height = ST7735_TFTWIDTH;
		break;
	}
}


void Adafruit_ST7735::invertDisplay(boolean i)
{
	writecommand(i ? ST7735_INVON : ST7735_INVOFF);
}

