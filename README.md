# ILI9341 SPI LCD library for STM32duino (STM official and libmaple F1, F4 core) 

This is an implementation of (yet another) Adafruit ILI9341 SPI lcd library for [STM32duino official STM core](https://github.com/stm32duino/Arduino_Core_STM32) and libmaple ([roger's](https://github.com/rogerclarkmelbourne/Arduino_STM32)  and [steve's mainly F4](https://github.com/stevstrong/Arduino_STM32)) core.

This version use various hardware features on STM32 F1xx (Cortex M3) and F4xx (Cortex M4). In particular,
this library uses the Arduino SPI (like) interface, in SPI.h and SPI.c bundled in the respective cores.
It uses hardware SPI provided by the API and where available DMA. In addition, it uses bit banding that maps
gpio register bits to memory locations.

When tested on [STM32F401ccu 'blackpill'](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2)
board, observed about 40%-50% performance improvements on 
[steve's libmaple F4 core](https://github.com/stevstrong/Arduino_STM32) vs say the 
Adafruit original libraries. However, the use of all these hardware features means that this library is
mcu (e.g. STM32 F1xx (Cortex M3) and F4xx (Cortex M4), etc) and core specific.
The [Adafruit's implementation](https://github.com/adafruit/Adafruit_ILI9341) 
is still a good library nevertheless and is pretty optimised as well for SPI TFT LCD and uses standard SPI interfaces.


## No warranty

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.   


### Build / Use

- note that the instructions here assumes familiarity with stm32duino libmaple and/or STM official core
  and the sketch firmware install methods (e.g. using st-link, serial etc)
[STM32duino official core wiki](https://github.com/stm32duino/wiki/wiki)
[STM32duino (roger's) libmaple core wiki](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki)
[STM32duino forum](https://www.stm32duino.com/index.php)

- In addition it also assumes familiarity with the ILI9341 lcd modules e.g.
  [2.2inch SPI Module ILI9341](http://www.lcdwiki.com/2.2inch_SPI_Module_ILI9341_SKU:MSP2202)  
  [Adafruit ILI9341 lcd tutorial](https://learn.adafruit.com/adafruit-2-8-and-3-2-color-tft-touchscreen-breakout-v2)
  \<- note this library is just for the graphics display, 'touch' handling isn't part of this.
  And this uses the SPI interface.


- note that the (libmaple roger's and steve's) core is a community core/effort and the official STM core is continually
being improved.  Hence things are always in flux including if those repositories and this would conntinue to exist, or if the codes would stay the way it is when this sketch/implementation is developed. there is no assurance if things would continue to work when you try this out

- the core needs to be [STM32duino libmaple core](https://github.com/rogerclarkmelbourne/Arduino_STM32)
  or [STM32duino official STM core](https://github.com/stm32duino/Arduino_Core_STM32)
  
- using ILI9341 lcds with SPI requires 6 gpio pins. The SPI pins has to go to the hardware SPI pins (normally SPI1
  default). check the mcu spec sheets on ST's web (e.g. 
  [stm32f103c8](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html)
  [stm32f401cc](https://www.st.com/en/microcontrollers-microprocessors/stm32f401cc.html) ) and board pins:
  - MOSI - for SPI  
  - MISO - for SPI  
  - SCK - for SPI  
  - CS pin - chip select  
  - DC pin - command / data control pin  
  - RST pin - LCD reset pin  
    
  You can connect an additional (gpio) pin to the backlight pin on the LCD module. However
  , do check that there is some resistance of a few hundred ohms  at the LCD backlight pin. 
  Accordingly, it goes to a resistor to transistor (base) there driving the backlight,  but it is beter to
  be careful so as not to short out mcu pins.  SPI uses the default SPI in the core (e.g. SPI1).
  
  the mappings in graphics test sketch is as follows
```
 #define TFT_CS			PA4
 #define TFT_CLK		PA5
 #define TFT_MISO		PA6
 #define TFT_MOSI		PA7
 #define TFT_RST		PB0
 #define TFT_DC			PB1

 #define BACKLIGHT		PB10
```
  
- A preferred way to build/test this out is to copy the following into a sketch folder and build from there. That would build with the bundled graphicstest.
  - Adafruit_GFX.cpp  
  - Adafruit_GFX.h
  - Adafruit_ILI9341_STM.h    
  - Adafruit_ILI9341_STM.cpp  
  - glcdfont.c
  - gfxfont.h 
  - graphicstest.ino  

In addition you need to include "SPI.h" and SPI library (SPI.h, SPI.cpp) from the library path.
  
- the graphics test sketch pauses for a keypress on the serial monitor (usb serial) before starting.
It responds to some single key commands.  
pressing 's' pauses at the current test.  
Once paused it responds to the following key commands '00' to '11' runs the particular test.  
'i' in paused mode runs tft.begin() and reinitialise the lcd and start tests.

- To use this library in your sketch
```
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341_STM.h"
#include <SPI.h>

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

tft.begin();
...
```
for the rest you may like to review the graphicstest sketch example.

### Tests and results

While the graphicstest sketch runs, it should display some values read from the registers. e.g.
```
ILI9341 Test!
Display Power Mode: 0x9C
MADCTL Mode: 0x48
Pixel Format: 0x5
Image Format: 0x9C
Self Diagnostic: 0xC0
Display id: 0x0,0,0
Display status: 0x29,82,0,0,0
```
if these values are zeros. check your assignment of the lcd reset pin and that the reset pin is well connected.
if the display remain blank afterwards, check all other pins as well. if the display is dark connect a 3.3v, or
5v to the lcd backlight pin, but check that there is a series resistance at the pin or connect one say 300 ohms.

Sample run on [STM32F401ccu black pill](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2) board running [steve's libmaple F4](https://github.com/stevstrong/Arduino_STM32) core

[![sample run STM32F01ccu steve libmaple core](http://img.youtube.com/vi/7Ey1U36YtY0/0.jpg)](http://www.youtube.com/watch?v=7Ey1U36YtY0)

some results:

Benchmark (microseconds) | Adafruit stm32f401ccu libmaple | this stm32f401ccu libmaple | this stm32f401ccu STM | this stm32f103c libmaple
--- | --- | --- | --- | --- 
Screen fill	| 476799 | 154057 |270212 | 180352
Text | 41574 | 28803 | 64929 | 46125
Lines | 352653 | 172626 | 419343 | 262946
Horiz/Vert Lines | 41711 | 15210 | 28710 | 19460
Rectangles (outline) | 26464 | 10875 | 21287 | 14081
Rectangles (filled) | 990327 | 321102 | 566554 | 376639
Circles (filled) | 138760 | 91825 | 206731 | 125040
Circles (outline) | 156673 | 105738 | 262278 | 183625
Triangles (outline) | 79426 | 46922 | 112643 | 65977
Triangles (filled) | 339544 | 152668 | 305522 | 202842
sum | 2643931 | 1099826 | 2258209 | 1477087

header legend
- first word. e.g. [Adafruit](https://github.com/adafruit/Adafruit_ILI9341) is the lib used, 'this' is this lib
- 2nd word: [stm32f401ccu](https://stm32-base.org/boards/STM32F401CCU6-WeAct-Black-Pill-V1.2)
 [stm32f103c](https://stm32-base.org/boards/STM32F103C8T6-Maple-Mini-Clone)
 is the mcu and board 
- 3rd word: is the core [STM](https://github.com/stm32duino/Arduino_Core_STM32), [libmaple](https://github.com/rogerclarkmelbourne/Arduino_STM32)

Note that this probably isn't the most 'optimized', but that an attempt is to only use the SPI.h interfaces. 
This in part to allow codes to be used across the cores, keeping much of the codes common. reducing the if-defs.
The use of bit banding still helps with the STM core, though it isn't too obvious in the above stats.
The Adafruit stats on libmaple core isn't directly comparable to STM core as it is different.

### Limitations

- This is 'alpha' software for now as it is pretty 'new' and 'unused'. So it may be 'broken' as you
  try this out.  It may be upgraded  to 'beta' at some point and it'd stay that way. things are always
  in flux and it is uncertain how far it'd remain that way.
- some 'corners' are cut, e.g. that additional SPI.beginTransaction() calls are not done
  in favor of a minor improvement in speeds. This could break, e.g. SPI in an inconsistent state
  if multiple SPI pheriperials are used.
- This library may not work on many core / mcu combinations. the board is normally irrelevant but listed as a reference. current known working combinations: 

core | mcu | freq | board
--- | --- | --- | ---
libmaple (rogers's, steve's) | stm32f401ccu | 84mhz | blackpill
libmaple (rogers's, steve's) | stm32f103c{8,b} | 72mhz | maple mini clone
STM official | stm32f401ccu | 84mhz | blackpill


### Author and credits 

This implementation is brought to you by Andrew Goh.
  
Many thanks goes to Adafruit for the [Adafruit GFX library](https://github.com/adafruit/Adafruit-GFX-Library).
this library (the contained Adafruit_ILI9341_STM.h and Adafruit_ILI9341_STM.cpp) is based on Adafruit GFX.
A copy of [Adafruit GFX library](https://github.com/adafruit/Adafruit-GFX-Library) is bundled with this library
as libraries tend to be updated over time and may break compatibility. However, that means this copy may be
outdated over time and do not benefit to fixes, improvements made in the [original Adafruit GFX library](https://github.com/adafruit/Adafruit-GFX-Library). 

And the various individuals Fedreric who maintain the Official (stm32duino) STM core and, Roger, Steve, and many others et\.al for the community libmaple core. it is a very impressive community effort with no big 'company' behind it. all that made from comunity contributions (including this implementation) that literally made this possible. 

- [Adafruit GFX library](https://github.com/adafruit/Adafruit-GFX-Library).
- [STM32duino official STM core](https://github.com/stm32duino/Arduino_Core_STM32)
- [STM32duino (roger's) libmaple core github](https://github.com/rogerclarkmelbourne/Arduino_STM32)
 note that this core contains the F4 implementation from steve as well. Hence, you could simply use this.
- [STM32duino libmaple core - steve's repository github](https://github.com/stevstrong/Arduino_STM32)
- [leaflabs libjmaple - the 'oldest' origin](http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/latest/libmaple.html)
- [STM32duino forum](https://www.stm32duino.com/index.php)
- [STM32duino (roger's) libmaple core wiki](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki)
- [STM32duino official core](https://github.com/stm32duino/Arduino_Core_STM32)
- [STM32duino official core wiki](https://github.com/stm32duino/wiki/wiki)
- [arduino IDE](https://www.arduino.cc/en/Main/Software)

developing this has taken quite some effort mainly in getting it to work across the cores, optimizations and some tests.
if you use this codes and found it useful, you may like to buy me a coffee [![Donate](resources/donorbox.svg)](https://donorbox.org/stm32duino-ili9341-lib) ;)