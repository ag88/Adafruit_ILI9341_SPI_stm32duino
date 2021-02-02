// See rights and use declaration in license.txt
// This library was modified to support DMA in Maple Mini by Victor Perez in 03/17/2015
// This library was modified (optimised) for the stm32duino libmaple (roger's) (F1 and F4) core and official STM core
// Andrew Goh 2021
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

#include "Adafruit_ILI9341_STM.h"
#include <Arduino.h>


// Constructor when using hardware SPI.  Faster, but must use SPI pins
Adafruit_ILI9341_STM::Adafruit_ILI9341_STM(int8_t cs, int8_t dc, int8_t rst)
: Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT)
{
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
}


void Adafruit_ILI9341_STM::writecommand(uint8_t c)
{
  dc_command();
  cs_clear();
  spiwrite(c);
  dc_data();
}


void Adafruit_ILI9341_STM::begin(SPIClass & spi, uint32_t freq)
{
  mSPI = spi;
  _freq = freq;
  _safe_freq = (freq>SAFE_FREQ) ? SAFE_FREQ : _freq;

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

#ifdef ARDUINO_ARCH_STM32
  mSPI.begin(_cs);
#endif

#ifdef ARDUINO_ARCH_STM32
  __IO uint32_t *base = portOutputRegister(digitalPinToPort(_cs));
  cs_addr = bb_perip(base, STM_PIN(digitalPinToPinName(_cs)));
  base = portOutputRegister(digitalPinToPort(_dc));
  dc_addr = bb_perip(base, STM_PIN(digitalPinToPinName(_dc)));
#elif defined(ARDUINO_ARCH_STM32F1)
  volatile uint32 *base = &(PIN_MAP[_cs].gpio_device->regs->ODR);
  cs_addr = bb_perip(base, PIN_MAP[_cs].gpio_bit);
  base = &(PIN_MAP[_dc].gpio_device->regs->ODR);
  dc_addr = bb_perip(base, PIN_MAP[_dc].gpio_bit);
#elif defined(ARDUINO_ARCH_STM32F4)
  volatile uint32 *base = portOutputRegister(digitalPinToPort(_cs));
  cs_addr = bb_perip(base, digitalPinToBit(_cs));
  base    = portOutputRegister(digitalPinToPort(_dc));
  dc_addr = bb_perip(base, digitalPinToBit(_dc));
#endif

  cs_set(); // deactivate chip

/*
 * initialise SPI bulk transfer settings
 * 8 bits/byte - this is safest common denominator, hardly any impact on speeds
 */
#ifdef ARDUINO_ARCH_STM32
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));
#elif defined(ARDUINO_ARCH_STM32F1)
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT));
#elif defined(ARDUINO_ARCH_STM32F4)
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
#endif

  // toggle RST low to reset
  if (_rst >= 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(100);
    digitalWrite(_rst, HIGH);
    delay(10);
  }

  /*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */

  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);
  writedata(0x00);
  writedata(0XC1);
  writedata(0X30);

  writecommand(0xED);
  writedata(0x64);
  writedata(0x03);
  writedata(0X12);
  writedata(0X81);

  writecommand(0xE8);
  writedata(0x85);
  writedata(0x00);
  writedata(0x78);

  writecommand(0xCB);
  writedata(0x39);
  writedata(0x2C);
  writedata(0x00);
  writedata(0x34);
  writedata(0x02);

  writecommand(0xF7);
  writedata(0x20);

  writecommand(0xEA);
  writedata(0x00);
  writedata(0x00);

  writecommand(ILI9341_PWCTR1);    //Power control
  writedata(0x23);   //VRH[5:0]

  writecommand(ILI9341_PWCTR2);    //Power control
  writedata(0x10);   //SAP[2:0];BT[3:0]

  writecommand(ILI9341_VMCTR1);    //VCM control
  writedata(0x3e);
  writedata(0x28);

  writecommand(ILI9341_VMCTR2);    //VCM control2
  writedata(0x86);  //--

  writecommand(ILI9341_MADCTL);    // Memory Access Control
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);
  writedata(0x55);

  writecommand(ILI9341_FRMCTR1);
  writedata(0x00);
  writedata(0x18);

  writecommand(ILI9341_DFUNCTR);    // Display Function Control
  writedata(0x08);
  writedata(0x82);
  writedata(0x27);

  writecommand(0xF2);    // 3Gamma Function Disable
  writedata(0x00);

  writecommand(ILI9341_GAMMASET);    //Gamma curve selected
  writedata(0x01);

  writecommand(ILI9341_GMCTRP1);    //Set Gamma
  writedata(0x0F);
  writedata(0x31);
  writedata(0x2B);
  writedata(0x0C);
  writedata(0x0E);
  writedata(0x08);
  writedata(0x4E);
  writedata(0xF1);
  writedata(0x37);
  writedata(0x07);
  writedata(0x10);
  writedata(0x03);
  writedata(0x0E);
  writedata(0x09);
  writedata(0x00);

  writecommand(ILI9341_GMCTRN1);    //Set Gamma
  writedata(0x00);
  writedata(0x0E);
  writedata(0x14);
  writedata(0x03);
  writedata(0x11);
  writedata(0x07);
  writedata(0x31);
  writedata(0xC1);
  writedata(0x48);
  writedata(0x08);
  writedata(0x0F);
  writedata(0x0C);
  writedata(0x31);
  writedata(0x36);
  writedata(0x0F);

  writecommand(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  writecommand(ILI9341_DISPON);    //Display on
  delay(120);
  cs_set();

  _width  = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));

}

void Adafruit_ILI9341_STM::startWrite(void) {
	//SPI_BEGIN_TRANSACTION();
	//mSPI.beginTransaction(_settings[0]);
	cs_clear();
}

void Adafruit_ILI9341_STM::endWrite(void) {
	cs_set();
	//SPI_END_TRANSACTION();
	//mSPI.endTransaction();
}


void Adafruit_ILI9341_STM::setAddrWindow(uint16_t x0, uint16_t y0,
                                         uint16_t x1, uint16_t y1)
{
  writecommand(ILI9341_CASET); // Column addr set
  spiwrite16(x0);
  spiwrite16(x1);
  
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite16(y0);
  spiwrite16(y1);

  writecommand(ILI9341_RAMWR); // write to RAM
}


/*
 * colors are 16 bits per pixel, using 8 bits (byte size) SPI transfers
 * hence nr_pixels * 2. SPI need to be setup before calling this.
 */
void Adafruit_ILI9341_STM::pushColors(void * colorBuffer, uint16_t nr_pixels, uint8_t async)
{
cs_clear();

#ifdef ARDUINO_ARCH_STM32
  mSPI.transfer(colorBuffer, nr_pixels*2);
#elif defined(ARDUINO_ARCH_STM32F1)
  if (async)
	  mSPI.dmaSendAsync(colorBuffer, nr_pixels*2, true);
  else
	  mSPI.dmaSend(colorBuffer, nr_pixels*2, true);
  #elif defined(ARDUINO_ARCH_STM32F4)
  if (async)
	  mSPI.dmaSend(colorBuffer, nr_pixels*2, DMA_ASYNC);
  else
	  mSPI.dmaSend(colorBuffer, nr_pixels*2, 0);
#endif

cs_set();
}

void Adafruit_ILI9341_STM::pushcolors(uint16_t color, uint32_t nr_pixels, uint8_t async){

	//swap byte order as this is using 8 bit SPI transfers
	color = ((color & 0xff00) >> 8) | ((color& 0xff) << 8);

	/* fill up color buffer */
    uint16_t blen = nr_pixels < COLORBUF_SIZE? nr_pixels :COLORBUF_SIZE;
	for(uint16_t i=0; i<blen; i++)
	  linebuffer[i] = color;

#ifdef ARDUINO_ARCH_STM32
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_TRANSMITONLY));
#endif

	/* push pixels */
	while(nr_pixels > 0) {
	  uint16_t nsend = nr_pixels < COLORBUF_SIZE? nr_pixels :COLORBUF_SIZE;
	  pushColors(linebuffer, nsend, async);

	  nr_pixels -= nsend;
	}

}

void Adafruit_ILI9341_STM::pushColor(uint16_t color)
{
  cs_clear();
  spiwrite16(color);
  cs_set();
}

void Adafruit_ILI9341_STM::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x, y, x + 1, y + 1);

  spiwrite16(color);

  cs_set();
}


void Adafruit_ILI9341_STM::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                        uint16_t color)
{
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || h < 1)) return;
  if ((y + h - 1) >= _height)
    h = _height - y;
  if (h < 2 ) {
    drawPixel(x, y, color);
    return;
  }

  setAddrWindow(x, y, x, y + h - 1);

  pushcolors(color, h, 0);

}


void Adafruit_ILI9341_STM::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                        uint16_t color)
{

  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width - x;
  if (w < 2 ) {
    drawPixel(x, y, color);
    return;
  }

  setAddrWindow(x, y, x + w - 1, y);

  pushcolors(color, w, 0);

}

void Adafruit_ILI9341_STM::fillScreen(uint16_t color)
{
   color = ((color & 0xff00) >> 8) | ((color& 0xff) << 8);

  for(uint16_t i=0; i<COLORBUF_SIZE;i++)
	  linebuffer[i] = color;
  setAddrWindow(0, 0, _width - 1, _height - 1);

  uint32_t nr_pix = _width * _height;

  pushcolors(color, nr_pix, 0);

}

// fill a rectangle
void Adafruit_ILI9341_STM::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   uint16_t color)
{

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height || h < 1 || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width  - x;
  if ((y + h - 1) >= _height) h = _height - y;
  if (w == 1 && h == 1) {
    drawPixel(x, y, color);
    return;
  }
  setAddrWindow(x, y, x + w - 1, y + h - 1);
  uint32_t nr_pix = w * h;

  pushcolors(color, nr_pix, 0);

}

/*
* Draw lines faster by calculating straight sections and drawing them with fastVline and fastHline.
*/
void Adafruit_ILI9341_STM::drawLine(int16_t x0, int16_t y0,
                                    int16_t x1, int16_t y1, uint16_t color)
{
	if ((y0 < 0 && y1 <0) || (y0 > _height && y1 > _height)) return;
	if ((x0 < 0 && x1 <0) || (x0 > _width && x1 > _width)) return;
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;

	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		}
		else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		}
		else {
			drawPixel(x0, y0, color);
		}
		return;
	}
	else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		}
		else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	if (steep) {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine (y0, xbegin, len + 1, color);
					//writeVLine_cont_noCS_noFill(y0, xbegin, len + 1);
				}
				else {
					drawPixel(y0, x0, color);
					//writePixel_cont_noCS(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeVLine_cont_noCS_noFill(y0, xbegin, x0 - xbegin);
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	}
	else {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
					//writeHLine_cont_noCS_noFill(xbegin, y0, len + 1);
				}
				else {
					drawPixel(x0, y0, color);
					//writePixel_cont_noCS(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeHLine_cont_noCS_noFill(xbegin, y0, x0 - xbegin);
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9341_STM::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341_STM::setRotation(uint8_t m)
{
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
    case 0:
      m = (MADCTL_MX | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      m = (MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 2:
      m = (MADCTL_MY | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
  }
  writecommand(ILI9341_MADCTL);
  writedata(m);
  cs_set();
}


void Adafruit_ILI9341_STM::invertDisplay(bool i)
{
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  cs_set();
}


uint16_t Adafruit_ILI9341_STM::readPixel(int16_t x, int16_t y)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));

  writecommand(ILI9341_CASET); // Column addr set
  spiwrite16(x);
  spiwrite16(x);
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite16(y);
  spiwrite16(y);
  writecommand(ILI9341_RAMRD); // read GRAM
  (void)spiread();             //dummy read
  uint8_t r = spiread();
  uint8_t g = spiread();
  uint8_t b = spiread();
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));

  return color565(r, g, b);
}

uint16_t Adafruit_ILI9341_STM::readPixels(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t *buf)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));

  writecommand(ILI9341_CASET); // Column addr set
  spiwrite16(x1);
  spiwrite16(x2);
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite16(y1);
  spiwrite16(y2);
  writecommand(ILI9341_RAMRD); // read GRAM
  (void)spiread();             //dummy read
  uint8_t r, g, b;
  uint16_t len = (x2-x1+1)*(y2-y1+1);
  uint16_t ret = len;
  while (len--) {
    r = spiread();
    g = spiread();
    b = spiread();
    *buf++ = color565(r, g, b);
  }
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
  return ret;
}


uint16_t Adafruit_ILI9341_STM::readPixelsRGB24(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t *buf)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));

  writecommand(ILI9341_CASET); // Column addr set
  spiwrite16(x1);
  spiwrite16(x2);
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite16(y1);
  spiwrite16(y2);
  writecommand(ILI9341_RAMRD); // read GRAM
  (void)spiread();             //dummy read
  uint16_t len = (x2-x1+1)*(y2-y1+1);
  uint16_t ret = len;

	for (int i = 0; i < len*3; i++) {
		uint8_t b = SPI.transfer(0x00);
		*(buf+i) = b;
	}

  //mSPI.dmaTransfer(buf, buf, len*3);
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
  return ret;
}

uint8_t Adafruit_ILI9341_STM::readcommand8(uint8_t c, uint8_t index)
{
  //writecommand(0xD9);
  //writedata(0x10+index);

  // the SPI clock must be set lower
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));

  writecommand(c);
  uint8_t r = spiread();
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
  return r;
}

void Adafruit_ILI9341_STM::readcommandx(uint8_t *buf, uint8_t c, int8_t n) {
	//uint index = 0;
	mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0));

	//writecommand(0xD9);
	//writedata(0x10+index);
	/*
	SETDC_CMD();
	CS_ENABLE();
	spiwrite(0xD9);  // woo sekret command?
	SETDC_DATA();
	spiwrite(0x10 + index);
	CS_DISABLE();
	*/

	writecommand(c);
	SPI.transfer(0x00); //the first 8 bits/clocks are dummy clocks
	for (int i = 0; i < n; i++) {
		uint8_t b = SPI.transfer(0x00);
		*(buf+i) = b;
	}

	cs_set();
	mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));
}


#define BB_PERI_REF      0x40000000
#define BB_PERI_BASE     0x42000000
volatile uint32_t* Adafruit_ILI9341_STM::bb_perip(volatile void *address, uint8_t bit) {
	return (volatile uint32_t *)(BB_PERI_BASE + ((uint32_t) address - BB_PERI_REF) * 32 + bit * 4);
}

/*

 uint16_t Adafruit_ILI9341_STM::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);

 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 uint32_t Adafruit_ILI9341_STM::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!

 dummyclock();
 dummyclock();

 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 */
