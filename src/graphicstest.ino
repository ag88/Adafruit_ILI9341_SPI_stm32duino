/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
 ****************************************************/
/*
 * Some edits to provide a more interactive response with serial commands
 * during the runs
 *
 * Andrew Goh 2021
 * released under the same MIT license and no warranty terms
 */

#include <Arduino.h>

#ifdef ARDUINO_ARCH_STM32
#include <USBSerial.h>
#define Serial SerialUSB
#elif defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F4)
#include <usb_serial.h>
#endif

#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341_STM.h"
#include <SPI.h>

#if defined(LED_BUILTIN)
#define	BOARD_LED_PIN LED_BUILTIN
#endif

unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
void sleep(int count);
void diag();

#define TFT_CS		PA4
#define TFT_CLK		PA5
#define TFT_MISO	PA6
#define TFT_MOSI	PA7
#define TFT_RST		PB0
#define TFT_DC		PB1

#define BACKLIGHT	PB10


// This use hardware SPI and the above pins for CS/DC and lcd reset pin
// the LCD reset pin is important, the display often can't initialise if
// lcd reset pin is not setup or connected correctly
Adafruit_ILI9341_STM tft = Adafruit_ILI9341_STM(TFT_CS, TFT_DC, TFT_RST);

bool blink = false;

uint8_t testn = 0;

void setup() {
  uint8_t c = 0;
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, LOW);

  while(!Serial.available()) sleep(1);
  Serial.read();

//  while(c != ' ') {
//	  c = Serial.read();
//	  sleep(1);
//  }

  Serial.begin();
  Serial.println("ILI9341 Test!"); 

  tft.begin();

  pinMode(BACKLIGHT, OUTPUT);
  digitalWrite(BACKLIGHT, HIGH);
}

bool bpause = false;

void loop(void) {

	if(Serial.available() || bpause) {
		uint8_t c = Serial.read();
		if (c == 's' || bpause) {
			while (!Serial.available()) sleep(1);
			uint8_t c = Serial.read();
			if(c >= '0' && c <= '9') {
				testn = c - '0';
				while (!Serial.available()) sleep(1);
				c = Serial.read();
				if(c >= '0' && c <= '9') {
					testn *= 10;
					testn += c - '0';
				}
				bpause = true;
			} else if(c == 'i') {
				tft.begin();
				testn = 0;
				bpause = false;
			} else
				bpause = false;
		}
		//flush input
		while(Serial.available()) Serial.read();
	}


	if (testn == 0) {

	  // read diagnostics (optional but can help debug problems)
	  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
	  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
	  x = tft.readcommand8(ILI9341_RDMADCTL);
	  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
	  x = tft.readcommand8(ILI9341_RDPIXFMT);
	  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
	  x = tft.readcommand8(ILI9341_RDIMGFMT);
	  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
	  x = tft.readcommand8(ILI9341_RDSELFDIAG);
	  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

	  diag();

	  Serial.println(F("Benchmark                Time (microseconds)"));
	  sleep(10);

	} else if (testn == 1) {
	  Serial.print(F("Screen fill              "));
	  Serial.println(testFillScreen());
	  //sleep(500);
	  //bpause = true;
	} else if (testn == 2) {
	  Serial.print(F("Text                     "));
	  Serial.println(testText());
	  //sleep(3000);
	} else if (testn == 3) {
	  Serial.print(F("Lines                    "));
	  Serial.println(testLines(ILI9341_CYAN));
	  //sleep(500);
	} else if (testn == 4) {
	  Serial.print(F("Horiz/Vert Lines         "));
	  Serial.println(testFastLines(ILI9341_RED, ILI9341_BLUE));
	  //sleep(500);
	} else if (testn == 5) {
	  Serial.print(F("Rectangles (outline)     "));
	  Serial.println(testRects(ILI9341_GREEN));
	  //sleep(500);
	} else if (testn == 6) {
	  Serial.print(F("Rectangles (filled)      "));
	  Serial.println(testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
	  //sleep(500);
	} else if (testn == 7) {
	  Serial.print(F("Circles (filled)         "));
	  Serial.println(testFilledCircles(10, ILI9341_MAGENTA));
	  //sleep(500);
	} else if (testn == 8) {
	  Serial.print(F("Circles (outline)        "));
	  Serial.println(testCircles(10, ILI9341_WHITE));
	  //sleep(500);
	} else if (testn == 9) {
	  Serial.print(F("Triangles (outline)      "));
	  Serial.println(testTriangles());
	  //sleep(500);
	} else if (testn == 10) {
	  Serial.print(F("Triangles (filled)       "));
	  Serial.println(testFilledTriangles());
	  //sleep(500);
	} else if (testn == 10) {
	  Serial.print(F("Rounded rects (outline)  "));
	  Serial.println(testRoundRects());
	  //sleep(500);
	} else if (testn == 10) {
	  Serial.print(F("Rounded rects (filled)   "));
	  Serial.println(testFilledRoundRects());
	  //sleep(500);
	} else if (testn == 11) {
	  for(uint8_t rotation=0; rotation<4; rotation++) {
	    tft.setRotation(rotation);
	    testText();
	  }
	} else if (testn == 12) {
		testn = 0;
		Serial.println(F("Done!"));
	} //else

    blink = ! blink;
    if(blink)
    	digitalWrite(BOARD_LED_PIN, LOW);
    else
    	digitalWrite(BOARD_LED_PIN, HIGH);
    sleep(500);
    testn++;
}

void diag() {
	uint8_t cmdbuf[10];
	// read diagnostics (optional but can help debug problems)

	tft.readcommandx(cmdbuf, ILI9341_RDDID, 4);
	Serial.print("Display id: 0x");
	for(int i=0;i<3;i++){
		if(i>0) Serial.print(',');
		Serial.print(cmdbuf[i], HEX);
	}
	Serial.println();

	tft.readcommandx(cmdbuf, ILI9341_RDDST,5);
	Serial.print("Display status: 0x");
	for(int i=0;i<5;i++){
		if(i>0) Serial.print(',');
		Serial.print(cmdbuf[i], HEX);
	}
	Serial.println();

	tft.readcommandx(cmdbuf, ILI9341_RDMODE,2);
	Serial.print("Display Power Mode: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);

	tft.readcommandx(cmdbuf, ILI9341_RDMADCTL,2);
	Serial.print("MADCTL Mode: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);

	tft.readcommandx(cmdbuf, ILI9341_RDPIXFMT,2);
	Serial.print("Pixel Format: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);

	Serial.println("set pix format 16 bits");
	tft.writecommand(ILI9341_PIXFMT);
	tft.writedata(0x55);

	tft.readcommandx(cmdbuf, ILI9341_RDPIXFMT,2);
	Serial.print("Pixel Format: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);

	tft.readcommandx(cmdbuf, ILI9341_RDIMGFMT, 2);
	Serial.print("Image Format: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);

	/*
	tft.readcommandxx(cmdbuf, ILI9341_RDSELFDIAG, 1);
	Serial.print("Self Diagnostic: 0x");
	Serial.println(cmdbuf[0], HEX);

	Serial.println("executing sleep in");
	tft.writecommand(ILI9341_SLPIN);
	delay(10);
	Serial.println("executing sleep out");
	tft.writecommand(ILI9341_SLPOUT);
	delay(500);
	*/

	tft.readcommandx(cmdbuf, ILI9341_RDSELFDIAG, 2);
	Serial.print("Self Diagnostic: 0x");
	Serial.println(cmdbuf[0], HEX);
	Serial.println(cmdbuf[1], HEX);


}


unsigned long testFillScreen() {
  uint16_t colors[5] = {
	ILI9341_BLACK,
	ILI9341_RED,
	ILI9341_GREEN,
	ILI9341_BLUE,
	ILI9341_BLACK
  };
  unsigned long start = micros();
  for(int i=0; i< 5; i++) {
	tft.fillScreen(colors[i]);
	yield();
	//sleep(1000);
  }
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}

/*
 * "wfi" is wait-for-interrupt, systick triggers every millisecond
 * and maybe there is usb start of frames interrupts etc. that would wakeup
 * this and increase the count. this keeps the mcu running cool ;)
 */
void sleep(int count) {
	for(int i=0; i<count; i++) {
		asm("wfi");
	}
}

