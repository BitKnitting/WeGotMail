// GotMail_MB_V1
// This code is loaded in the RFM95 LoRa Feather that is put into the mailbox.  It's job
// is to let the GotMail_Home unit know when Randy has opened the mail box and put mail in.
// This way, we'll know when mail has arrived because the GotMail_Home device will make
// us aware that we've got mail.
// Copyright: Margaret Johnson, 2017
// License: MIT License (https://opensource.org/licenses/MIT)
// Please give this work credit if you evolve and use.
// Uses Radio Head's reliable RFM95 packet library:
// http://www.airspayce.com/mikem/arduino/RadioHead/index.html
// Logic:
// The chips are (hopefully deeply) sleeping to save as much battery power as possible.  This
// way we'll minimize how often we change the batteries.  (note: We could set up a solar panel
// and feed the wire into the mailbox.  However, our mailbox is within a mailbox structure on
// our neighbor's property.  Unless, the solar panel is completely obscure - hey what about
// being on the lid of the mail box?  It might not be "a good thing" to have...so for now...
// no solar).  When our mail person opens, the door of the mailbox, the motion detector triggers
// an interrupt.  This event wakes up the m0 (from my knowledge the RF95 radio will wake up on
// a send/receive request).  We send a "You've got mail" message to the GotMail_Home unit to
// handle.  We'll try for a minute to send the You've got mail message.  When we get an ACK
// from the GotMail_Home device or a minute has gone by without a reply from the GotMail_Home
// device, we go back to sleep.  Knowing that tomorrow most likely will bring us more mail
// stuff of great importance...like spam catalogs and all the rest of the mail that contributes
// to our carbon footprint.
// the GotMail_Home device, we go back to sleep.
/****************** RF95 ***************************************/
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
RHReliableDatagram rf95_manager(rf95, MAIL_BOX_ADDRESS);
/****************** MOTION DETECTION ***************************************/
// Arduino's AttachInterrupt page: https://www.arduino.cc/en/Reference/AttachInterrupt
// notes all digital pins, except 4 can be used with attachInterrupt() (see Zero)
// Adafruit's Feather m0 RF95 page: https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/pinouts
// shows the pins that are available.  From above, pins 8, 3, 4, and LED_BUILTIN (which is #13) are taken.
// we are using a PIR Sensor Rev B https://www.parallax.com/product/555-28027
const int motionPin = 12; // pin on the m0 that is hooked up to the motion detection sensor.
const uint8_t numSecondsToWaitForReply = 10; // we'll keep trying to send a message to the home device
bool bMotionDetected = false; // flag use to trigger motion detection action.  Triggered within interrupt handler.
// for up to a minute that the mailbox was just opened.  Once this time is up, we go back to sleep.
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
/****************** POWER MANAGEMENT ***************************************/
#define LOW_POWER
#ifndef LOW_POWER  // if we're not running in low power mode, assume a serial monitor for debugging.
#define DEBUG
#endif
#include <DebugLib.h>
/********************************************************
  SETUP
 ********************************************************/
void setup()
{
#ifdef LOW_POWER
  delay(10000);  // Could brick the m0 if the delay is not put in before putting chip to sleep.
  Blink(LED, 400, 3);
#endif
  DEBUG_BEGIN;
  DEBUG_WAIT;
  initStuff();
}
/********************************************************
  LOOP
  - when in low power mode, put the chips to sleep.
  - all the action happens when the motion detector sends an interrupt.
 ********************************************************/
void loop()
{
  goToSleep();
  // while blinking takes up a bit of current, I want to be able to do minimal debugging
  // when in low power.  I'd like to have a visual when the chip actually wakes up.
  if (bMotionDetected) {
    DEBUG_PRINTLNF("Motion is detected...");
    Blink(LED, 400, 3);
    bMotionDetected = false;
    sendGotMailMessage();
  }
}
/********************************************************
   INITSTUFF
   put all the init gunk here so it's easier to read what setup does.
 ********************************************************/
void initStuff() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); // save power since this sits in the mailbox.
  initMotionDetection();
  initRadio();
}
/********************************************************
   INITMOTIONDETECTION
 ********************************************************/
void initMotionDetection() {
  pinMode(motionPin, INPUT);     // declare sensor as input

  // When sending a gotMail message, the packetType doesn't change.  So set it once.
  gotMail.values.packetType = packetGotMail;
}
/********************************************************
  INITRADIO
  initialize the RFM69 radio.  I put it at maximum power because of the
  distance of the mailbox.  This may be overkill.  I might want to revisit
  if the battery life is such that I'm constantly having to recharge.
  The other thing is...I'm familiar with the RF69...this is the first time I've used
  The RF95 LoRa...there might be a better way to initialize.  I got this way mostly
  from Adafruit sample code:  https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
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
/********************************************************
   GOTOSLEEP
     I got this from:https://github.com/GabrielNotman/AutonomoTesting/blob/master/Interrupts/SleepChangeInterrupt/SleepChangeInterrupt.ino
   Then added based on p. 153 of Atmel SMART SAM D21 Datasheet (9/20/2015)
   standby - go into deep sleep and wake up on the Motion sensor interrupt
 ********************************************************/
void  goToSleep()
{
  //Attach the interrupt and set the wake flag
  // From Arduino's AttachInterupt page: https://www.arduino.cc/en/Reference/AttachInterrupt
  // The first parameter to attachInterrupt is an interrupt number.
  // Normally you should use digitalPinToInterrupt(pin) to translate
  // the actual digital pin to the specific interrupt number.
  attachInterrupt(digitalPinToInterrupt(motionPin), motionDetected, HIGH);
  // put the rf95 to sleep
  rf95.sleep();
#ifdef LOW_POWER

  // put the m0 to sleep.
  // I did not see any less current being used when I did this so commented out.
  // Set the XOSC32K to run in standby
  //  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
  //  SYSCTRL->XOSC32K.bit.ONDEMAND = 0;
  //  SYSCTRL->XOSC32K.bit.ENABLE = 1;
  //
  //  // Configure EIC to use GCLK1 which uses XOSC32K
  //  // This has to be done after the first call to attachInterrupt()
  //  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
  //                      GCLK_CLKCTRL_GEN_GCLK1 |
  //                      GCLK_CLKCTRL_CLKEN;
  //Set standby mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
#endif
}
/********************************************************
  MOTIONDETECTED
  A callback set up during init to associate the motionPin interrupt
  with a routine to execute when motion is detected.  When motion is
  detected, the You've got mail message is sent to the home device.

  The key is to do as little as possible within the ISR.  In this case,
  set a flag that will be evaluated in the loop().
 ********************************************************/
void motionDetected() {
  bMotionDetected = true;
}
/********************************************************
   SENDGOTMAILMESSAGE
   Motion has been detected on the mail box door.  We assume this means
   the mail person is putting mail into the box.  We send a GotMail message
   to the home device.  We try for numSecondsToWaitForReply
 ********************************************************/
void sendGotMailMessage() {
#ifdef LOW_POWER
  Blink(LED, 400, 3);
#endif
  // Detach the interrupt for motion detection while sending mail.  The motion detection interrupt keeps firing when packets are
  // set by the RF95.
  detachInterrupt(digitalPinToInterrupt(motionPin));
  unsigned long currentMillis = millis();
  unsigned long startingMillis = currentMillis;
  gotMail.values.batteryLevel = readBatteryLevel();
  while (currentMillis - startingMillis < numSecondsToWaitForReply * 1000) {
    DEBUG_PRINT("...sending gotMail packet.  The amount of millis that have passed: "); DEBUG_PRINTLN(currentMillis - startingMillis);
    if (rf95_manager.sendtoWait(gotMail.b, sizeof(gotMail), HOME_ADDRESS)) {
      DEBUG_PRINTLNF("...got an ACK...");
      break;
    }
    currentMillis = millis();
  }
  DEBUG_PRINTLNF("...back to waiting for motion...");
  // We've either timed out or got an ACK... we're getting sleepy...very sleepy....
  // we'll be going to sleep when the code gets back into the loop().
}
/********************************************************
   READBATTERYLEVEL
   // Code from Adafruit's learn section:https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/overview?view=all#measuring-battery
 ********************************************************/
#define VBATPIN A7
float readBatteryLevel() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return (measuredvbat);
}
/********************************************************
   BLINK
 ********************************************************/
void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
