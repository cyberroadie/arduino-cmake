//
// Created by Olivier Van Acker on 08/08/15.
//
#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "guggenhat.h"


/*--------------------------------------------------------------------------
  GUGGENHAT: a Bluefruit LE-enabled wearable NeoPixel marquee.

  Requires:
  - Arduino Micro or Leonardo microcontroller board.  An Arduino Uno will
    NOT work -- Bluetooth plus the large NeoPixel array requires the extra
    512 bytes available on the Micro/Leonardo boards.
  - Adafruit Bluefruit LE nRF8001 breakout: www.adafruit.com/products/1697
  - 4 Meters 60 NeoPixel LED strip: www.adafruit.com/product/1461
  - 3xAA alkaline cells, 4xAA NiMH or a beefy (e.g. 1200 mAh) LiPo battery.
  - Late-model Android or iOS phone or tablet running nRF UART or
    Bluefruit LE Connect app.
  - BLE_UART, NeoPixel, NeoMatrix and GFX libraries for Arduino.

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
  MIT license.  All text above must be included in any redistribution.
  --------------------------------------------------------------------------*/

//#include <SPI.h>
#include <adafruit_ble_uart/Adafruit_BLE_UART.h>
#include <adafruit_neopixel/Adafruit_NeoPixel.h>
#include <adafruit_neomatrix/Adafruit_NeoMatrix.h>
#include <adafruit_gfx/Adafruit_GFX.h>

// NEOPIXEL STUFF ----------------------------------------------------------

// 4 meters of NeoPixel strip is coiled around a top hat; the result is
// not a perfect grid.  My large-ish 61cm circumference hat accommodates
// 37 pixels around...a 240 pixel reel isn't quite enough for 7 rows all
// around, so there's 7 rows at the front, 6 at the back; a smaller hat
// will fare better.
#define NEO_PIN     6 // Arduino pin to NeoPixel data input
#define NEO_WIDTH  37 // Hat circumference in pixels
#define NEO_HEIGHT  7 // Number of pixel rows (round up if not equal)
#define NEO_OFFSET  (((NEO_WIDTH * NEO_HEIGHT) - 240) / 2)

// Pixel strip must be coiled counterclockwise, top to bottom, due to
// custom remap function (not a regular grid).
Adafruit_NeoMatrix matrix(NEO_WIDTH, NEO_HEIGHT, NEO_PIN,
                          NEO_MATRIX_TOP  + NEO_MATRIX_LEFT +
                          NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
                          NEO_GRB         + NEO_KHZ800);

char          msg[21]       = {0};            // BLE 20 char limit + NUL
uint8_t       msgLen        = 0;              // Empty message
int           msgX          = matrix.width(); // Start off right edge
unsigned long prevFrameTime = 0L;             // For animation timing
#define FPS 20                                // Scrolling speed

// BLUEFRUIT LE STUFF-------------------------------------------------------

// CLK, MISO, MOSI connect to hardware SPI.  Other pins are configrable:
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RST  9
#define ADAFRUITBLE_RDY  2 // Must be an interrupt pin

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(
        ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t  prevState  = ACI_EVT_DISCONNECTED;

// STATUS LED STUFF --------------------------------------------------------

// The Arduino's onboard LED indicates BTLE status.  Fast flash = waiting
// for connection, slow flash = connected, off = disconnected.
#define LED 13                   // Onboard LED (not NeoPixel) pin
int           LEDperiod   = 0;   // Time (milliseconds) between LED toggles
boolean       LEDstate    = LOW; // LED flashing state HIGH/LOW
unsigned long prevLEDtime = 0L;  // For LED timing

// UTILITY FUNCTIONS -------------------------------------------------------

// Because the NeoPixel strip is coiled and not a uniform grid, a special
// remapping function is used for the NeoMatrix library.  Given an X and Y
// grid position, this returns the corresponding strip pixel number.
// Any off-strip pixels are automatically clipped by the NeoPixel library.
uint16_t remapXY(uint16_t x, uint16_t y) {
    return y * NEO_WIDTH + x - NEO_OFFSET;
}

// Given hexadecimal character [0-9,a-f], return decimal value (0 if invalid)
uint8_t unhex(char c) {
    return ((c >= '0') && (c <= '9')) ?      c - '0' :
           ((c >= 'a') && (c <= 'f')) ? 10 + c - 'a' :
           ((c >= 'A') && (c <= 'F')) ? 10 + c - 'A' : 0;
}

// Read from BTLE into buffer, up to maxlen chars (remainder discarded).
// Does NOT append trailing NUL.  Returns number of bytes stored.
uint8_t readStr(char dest[], uint8_t maxlen) {
    int     c;
    uint8_t len = 0;
    while((c = BTLEserial.read()) >= 0) {
        if(len < maxlen) dest[len++] = c;
    }
    return len;
}

// MEAT, POTATOES ----------------------------------------------------------

void setup() {
    matrix.begin();
    matrix.setRemapFunction(remapXY);
    matrix.setTextWrap(false);   // Allow scrolling off left
    matrix.setTextColor(0xF800); // Red by default
    matrix.setBrightness(31);    // Batteries have limited sauce

    BTLEserial.begin();

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
}

void loop() {
    unsigned long t = millis(); // Current elapsed time, milliseconds.
    // millis() comparisons are used rather than delay() so that animation
    // speed is consistent regardless of message length & other factors.

    BTLEserial.pollACI(); // Handle BTLE operations
    aci_evt_opcode_t state = BTLEserial.getState();

    if(state != prevState) { // BTLE state change?
        switch(state) {        // Change LED flashing to show state
            case ACI_EVT_DEVICE_STARTED: LEDperiod = 1000L / 10; break;
            case ACI_EVT_CONNECTED:      LEDperiod = 1000L / 2;  break;
            case ACI_EVT_DISCONNECTED:   LEDperiod = 0L;         break;
        }
        prevState   = state;
        prevLEDtime = t;
        LEDstate    = LOW; // Any state change resets LED
        digitalWrite(LED, LEDstate);
    }

    if(LEDperiod && ((t - prevLEDtime) >= LEDperiod)) { // Handle LED flash
        prevLEDtime = t;
        LEDstate    = !LEDstate;
        digitalWrite(LED, LEDstate);
    }

    // If connected, check for input from BTLE...
    if((state == ACI_EVT_CONNECTED) && BTLEserial.available()) {
        if(BTLEserial.peek() == '#') { // Color commands start with '#'
            char color[7];
            switch(readStr(color, sizeof(color))) {
                case 4:                  // #RGB    4/4/4 RGB
                    matrix.setTextColor(matrix.Color(
                            unhex(color[1]) * 17, // Expand to 8/8/8
                            unhex(color[2]) * 17,
                            unhex(color[3]) * 17));
                    break;
                case 5:                  // #XXXX   5/6/5 RGB
                    matrix.setTextColor(
                            (unhex(color[1]) << 12) +
                            (unhex(color[2]) <<  8) +
                            (unhex(color[3]) <<  4) +
                            unhex(color[4]));
                    break;
                case 7:                  // #RRGGBB 8/8/8 RGB
                    matrix.setTextColor(matrix.Color(
                            (unhex(color[1]) << 4) + unhex(color[2]),
                            (unhex(color[3]) << 4) + unhex(color[4]),
                            (unhex(color[5]) << 4) + unhex(color[6])));
                    break;
            }
        } else { // Not color, must be message string
            msgLen      = readStr(msg, sizeof(msg)-1);
            msg[msgLen] = 0;
            msgX        = matrix.width(); // Reset scrolling
        }
    }

    if((t - prevFrameTime) >= (1000L / FPS)) { // Handle scrolling
        matrix.fillScreen(0);
        matrix.setCursor(msgX, 0);
        matrix.print(msg);
        if(--msgX < (msgLen * -6)) msgX = matrix.width(); // We must repeat!
        matrix.show();
        prevFrameTime = t;
    }
}
