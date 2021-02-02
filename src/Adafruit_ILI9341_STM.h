// See rights and use declaration in license.txt
// This library was modified (optimised) for the stm32duino libmaple (roger's) (F1 and F4) core and official STM core
// Andrew Goh 2021
//
// MIT license as below and no warranty terms applies
/*
This is our library for the Adafruit ILI9341 Breakout and Shield
----> http://www.adafruit.com/products/1651

Check out the links above for our tutorials and wiring diagrams
These displays use SPI to communicate, 4 or 5 pins are required to
interface (RST is optional)
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef _ADAFRUIT_ILI9341H_
#define _ADAFRUIT_ILI9341H_

#include <Arduino.h>
#include "Adafruit_GFX.h"
#include <SPI.h>

#ifndef swap
#define swap(a, b) { int16_t t = a; a = b; b = t; }
#endif

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0A
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR   0x30
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
 #define ILI9341_PWCTR6  0xFC

 */

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

class Adafruit_ILI9341_STM: public Adafruit_GFX {

public:

	/*
	 * using ILI9341 lcds requires 6 gpio pins
	 * (3 PINs for hardware SPI - MOSI, MISO, SCK
	 *
	 * CS pin - chip select
	 * DC pin - command / data control pin
	 * RST pin - LCD reset pin
	 *
	 * creating a new class with _RST = -1 would skip reset
	 * however, the LCD may not initialize properly and fail to work
	 *
	 * you can connect an additional (gpio) pin to the backlight pin on the LCD
	 * module. However, do check that there is some resistance of a few hundred ohms
	 * at the LCD pin. Accordingly, it goes to a resistor to transistor (base) there driving the backlight,
	 * but it is beter to be careful so as not to short out mcu pins.
	 */
	Adafruit_ILI9341_STM(int8_t _CS, int8_t _DC, int8_t _RST);

	/* initialization, call tft.begin() to initialize the LCD
	 * this will also trigger a lcd reset if reset pin > -1
	 * calling without arguments uses the default SPI interface (normally SPI1)
	 * the SPI frequency can optionally be specified
	 */
	void begin(SPIClass &spi, uint32_t freq = 48000000);
	void begin(void) {
		begin(SPI);
	}

	/*
	 * These are overrides of various graphics primitives from the Adafruit GFX
	 * graphics library. Adafruit GFX provides many more graphics primitives
	 * than just here. take a look at Adafruit_GFX.h
	 *
	 * https://learn.adafruit.com/adafruit-gfx-graphics-library
	 * https://github.com/adafruit/Adafruit-GFX-Library
	 *
	 * overriding these allow optimizations to happen. This is done here
	 * and it accelerates various graphics ops using hardware SPI and where available DMA
	 *
	 * Adafruit GFX calls some of these primitives.
	 */
	void fillScreen(uint16_t color);
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
			uint16_t color);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
	void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

	/* some ILI9341 specific feature functions */

	void setRotation(uint8_t r);
	void invertDisplay(bool i);

	/***  'internal' functions below ***/

	/* (Adafruit GFX calls) startWrite to control the CS pin (CS low - i.e. enable)
	 */
	void startWrite(void);
	/* (Adafruit GFX calls) endWrite to control the CS pin (CS high - i.e. disable)
	 */
	void endWrite(void);

	/* this method sets the 'address window' bounding box and gets it ready for a memory write
	 * this is called internally by the drawing functions to setup the window
	 */
	void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

	/* this is currently unused, it may be used, with setAddrWindow to 'draw a pixel' */
	void pushColor(uint16_t color);

	/* this stream pixels out using hardware SPI (with DMA if possible)
	 * it uses a linebuffer (defined in private variables below) that is COLORBUF_SIZE large.
	 */
	void pushColors(void *colorBuffer, uint16_t nr_pixels, uint8_t async = 0);

	/* this is used internally by fillScreen(), fillRect() to fill up the 'Address window'
	 * bounding box with color pixels.
	 * calls pushColors in turn to actually push the pixels
	 */
	void pushcolors(uint16_t color, uint32_t nr_pixels, uint8_t async = 0);

	/* some 'utility' functions */

	uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

	uint16_t readPixel(int16_t x, int16_t y);
	uint16_t readPixels(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
			uint16_t *buf);
	uint16_t readPixelsRGB24(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
			uint8_t *buf);

	/* 'low level' routines */
	uint8_t readcommand8(uint8_t reg, uint8_t index = 0);
	void readcommandx(uint8_t *buf, uint8_t reg, int8_t index);
	/*
	 uint16_t readcommand16(uint8_t);
	 uint32_t readcommand32(uint8_t);
	 */

	inline uint8_t spiread(void) {
		return mSPI.transfer(0x00);
	}
	inline uint8_t readdata(void) {
		return mSPI.transfer(0x00);
	}
	inline void writedata(uint8_t c) {
		mSPI.transfer(c);
	}
	inline void spiwrite(uint16_t c) {
		mSPI.transfer(c);
	}
	inline void spiwrite16(uint16_t c) {
		mSPI.transfer16(c);
	} // 8 bit mode

	void writecommand(uint8_t c);

	/* note bit banding is used here */
	inline void dc_command() { *dc_addr = 0; } // 0
	inline void dc_data()    { *dc_addr = 1; } // 1
	inline void cs_clear()   { *cs_addr = 0; }
	inline void cs_set()     { *cs_addr = 1; }

private:
#define SAFE_FREQ  24000000ul // 24MHz for reading
	uint32_t _freq, _safe_freq;
	SPIClass &mSPI = SPI;
	int8_t _cs, _dc, _rst;
	volatile uint32_t *dc_addr, *cs_addr;


// COLORBUF_SIZE has to be multiples of 2 as each pixel is 16 bit color
// e.g. if COLORBUF_SIZE is 64, each pixel is 16 bits (2 bytes)
// hence that makes it 64px * 2 ~ 128 bytes used in ram
#define COLORBUF_SIZE 64
	uint16_t linebuffer[COLORBUF_SIZE]; //DMA pixel buffer 16bit color data per pixel

	// this library uses bitbanding this means that it would only work on mcus with bit-banding
	// e.g. Cortex M3 (e.g. stm32f1xx), Cortex M4 (e.g. stm32f4xx) etc
	volatile uint32_t* bb_perip(volatile void *address, uint8_t bit);

};

#endif
