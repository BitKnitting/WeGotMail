// GotMail_Home_V2
// This code is loaded in the RFM95 LoRa Feather that sets at home.  It's job
// is to listen for a message from the GotMail_MB unit.  When it gets a message, it
// lets us know the mailbox door has been opened...which means we should have received mail.
// V2: Added low power so the GotMail_Home unit can run on a battery.
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.
// Uses Radio Head's reliable RFM95 packet library:
// http://www.airspayce.com/mikem/arduino/RadioHead/index.html

// RE: LOW POWER...I tried several things that would wake me up from a deep sleep when packet(s) came into the
// RFM95 (interrupt).  Sadly, I was not able to...so the low power logic is:
// - on boot,
//      if the time is between 11 and 6PM, go into idle mode 2 because this is the time period when we get mail.
//      if the time is not between 11 and 6PM, use the rtc_zero library to set an alarm for either 11AM or 6PM (depending
//      on what time it is).  We use rtc_zero because the library sets the m0 into deep sleep which gives a significant
//      boost to the battery life.
// - when the alarm goes off, put the m0 into idle mode 2.  In this state, we are waiting for the mail box to open.
// - when we get an interrupt that the mail box is open
// into idle mode.  When mail is delivered and the button is pushed, use the rtc_zero library to set an alarm for 11:30 and
// then go into deep sleep.  When the alarm goes off, go into idle mode 2.
//****************** RF95 ***************************************/
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define MAIL_BOX_ADDRESS 11
#define HOME_ADDRESS 12
#define RF95_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS      8
#define RFM95_INT     3
#define RFM95_RST     4
#define LED           LED_BUILTIN
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf95_manager(rf95, HOME_ADDRESS);
//***************************** mail box stuff *******************************
const uint8_t packetGotMail = 3;
struct gotMail_t
{
  uint8_t packetType;
  float   batteryLevel;
};
union gotMailUnion_t
{
  gotMail_t values;
  uint8_t b[sizeof(gotMail_t)];
} gotMail;
//****************** NEOPIXEL and BUTTON feedbook **************************
#include <Adafruit_NeoPixel.h>
// using const int because PIN is the
#define NeoPin 6 // see https://learn.adafruit.com/adafruit-neopixel-featherwing/pinouts
#define NUM_LEDS 32
#define BRIGHTNESS 50
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NeoPin, NEO_GRBW + NEO_KHZ800);
#define BUTTON 11 // pin for attaching button.
volatile bool bButtonPushed = false;
bool bSparkle = false;
#define LOW_BATTERY 3.7
// Convenient 2D point structure
struct Point {
  float x;
  float y;
};
//#define DEBUG
#include <DebugLib.h>

/********************************************************
   SETUP
 ********************************************************/
void setup()
{
  initStuff();
}
/********************************************************
   LOOP
   - go to sleep and wait for the RFM69 radio to be interrupted that a packet has come in from the GotMail_MB.
   - check to make sure the packet is a got mail packet.  If it is, turn on the neopixels so we can see the pretty lights.
   - note: there is a button that can be pushed to stop the pretty lights and put the GotMail_Home device back to sleep.
 ********************************************************/
void loop()
{
  if (rf95_manager.available()) // if RFM69 chip has incoming data...
  {
    Blink(500, 6);
    uint8_t len = sizeof(gotMail);
    uint8_t from;
    if (rf95_manager.recvfromAck(gotMail.b, &len, &from))  // receive it - should be a mailbox message...
    {
      if (gotMail.values.packetType == packetGotMail) {
        DEBUG_PRINTF("Got mail packet.  Battery level: "); DEBUG_PRINTLN(gotMail.values.batteryLevel);
        bSparkle = true;
        bButtonPushed = false;
      }
    }
  }
  if (bButtonPushed) {
#ifndef SERIAL
    Blink(500, 6);
#endif
    DEBUG_PRINTLNF("...button has been pushed...");
    bButtonPushed = false;
    bSparkle = false;
    colorWipe(); // turn off pixels
  }
  if (bSparkle) {
    sparkle();
  }
}
/********************************************************
   INITSTUFF
 ********************************************************/
void initStuff() {
  DEBUG_BEGIN;
  DEBUG_WAIT;
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON), handleButtonPush, HIGH);
  initNeopixel();
  initRadio();
  Blink(500, 5);
  digitalWrite(LED, HIGH);
}
/********************************************************
  INITNEOPIXEL
********************************************************/
void initNeopixel() {
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  colorWipe(); // turn off pixels
}
/********************************************************
  INITRADIO
  this is mostly copy/pasted from an Adafruit example
********************************************************/
void initRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95_manager.init()) {
    DEBUG_PRINTLNF("Could not initialize RH manager.");
    while (1) ;
  }
  rf95.setFrequency(RF95_FREQ);
  // from Adafruit's sample: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  DEBUG_PRINTF("RFM95 radio @");  DEBUG_PRINT((int)RF95_FREQ);  DEBUG_PRINTLNF(" MHz");
}
void handleButtonPush() {
  bButtonPushed = true;
}
/********************************************************
  COLORWIPE
  Turn off pixels
********************************************************/
// Fill the dots one after the other with a color
void colorWipe() {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
  }
}
/********************************************************
  SPARKLE
  See John Ericksen's work: https://raw.githubusercontent.com/johncarl81/neopixelplasma/master/neopixelplasma.ino
********************************************************/
void sparkle() {
  // if the mail box puck returned a low battery reading, do a different pattermn.
  if (gotMail.values.batteryLevel <= LOW_BATTERY) {
    Fire(55, 120, 50);
  } else {
    static float phase = 0.0;
    float phaseIncrement = 0.03;  // Controls the speed of the moving points. Higher == faster. I like 0.08 .
    float colorStretch = 0.3;    // Higher numbers will produce tighter color bands. I like 0.11 .
    phase += phaseIncrement;

    // The two points move along Lissajious curves, see: http://en.wikipedia.org/wiki/Lissajous_curve
    // We want values that fit the LED grid: x values between 0..13, y values between 0..8 .
    // The sin() function returns values in the range of -1.0..1.0, so scale these to our desired ranges.
    // The phase value is multiplied by various constants; I chose these semi-randomly, to produce a nice motion.
    Point p1 = { (sin(phase * 1.000) + 1.0) * 4.5, (sin(phase * 1.310) + 1.0) * 4.0 };
    Point p2 = { (sin(phase * 1.770) + 1.0) * 4.5, (sin(phase * 2.865) + 1.0) * 4.0 };
    Point p3 = { (sin(phase * 0.250) + 1.0) * 4.5, (sin(phase * 0.750) + 1.0) * 4.0 };

    byte row, col;

    // For each row...
    for ( row = 0; row < 5; row++ ) {
      float row_f = float(row);  // Optimization: Keep a floating point value of the row number, instead of recasting it repeatedly.

      // For each column...
      for ( col = 0; col < 8; col++ ) {
        float col_f = float(col);  // Optimization.

        // Calculate the distance between this LED, and p1.
        Point dist1 = { col_f - p1.x, row_f - p1.y };  // The vector from p1 to this LED.
        float distance1 = sqrt( dist1.x * dist1.x + dist1.y * dist1.y );

        // Calculate the distance between this LED, and p2.
        Point dist2 = { col_f - p2.x, row_f - p2.y };  // The vector from p2 to this LED.
        float distance2 = sqrt( dist2.x * dist2.x + dist2.y * dist2.y );

        // Calculate the distance between this LED, and p3.
        Point dist3 = { col_f - p3.x, row_f - p3.y };  // The vector from p3 to this LED.
        float distance3 = sqrt( dist3.x * dist3.x + dist3.y * dist3.y );

        // Warp the distance with a sin() function. As the distance value increases, the LEDs will get light,dark,light,dark,etc...
        // You can use a cos() for slightly different shading, or experiment with other functions. Go crazy!
        float color_1 = distance1;  // range: 0.0...1.0
        float color_2 = distance2;
        float color_3 = distance3;
        float color_4 = (sin( distance1 * distance2 * colorStretch )) + 2.0 * 0.5;

        // Square the color_f value to weight it towards 0. The image will be darker and have higher contrast.
        color_1 *= color_1 * color_4;
        color_2 *= color_2 * color_4;
        color_3 *= color_3 * color_4;
        color_4 *= color_4;

        // Scale the color up to 0..7 . Max brightness is 7.
        //strip.setPixelColor(col + (8 * row), strip.Color(color_4, 0, 0) );
        strip.setPixelColor(col + (8 * row), strip.Color(color_1, color_2, color_3));
      }
    }
    strip.show();
  }
}
/********************************************************
  FIRE
  Slight modification of Tweaking4All's work: https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#fire
********************************************************/
void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);

    if (cooldown > heat[i]) {
      heat[i] = 0;
    } else {
      heat[i] = heat[i] - cooldown;
    }
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if ( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160, 255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for ( int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j] );
  }

  strip.show();
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);

  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if ( t192 > 0x80) {                    // hottest
    strip.setPixelColor(Pixel, 255, 0, heatramp);
  } else if ( t192 > 0x40 ) {            // middle
    strip.setPixelColor(Pixel, 255, heatramp / 20, 0);
  } else {                               // coolest
    strip.setPixelColor(Pixel, 255, heatramp / 10,  0);
    //strip.setPixelColor(Pixel, heatramp, 0, 0);
  }
}

/********************************************************
   BLINK
 ********************************************************/
void Blink(byte DELAY_MS, byte loops) {
#ifndef DEBUG
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(LED, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED, LOW);
    delay(DELAY_MS);
  }
#endif
}

