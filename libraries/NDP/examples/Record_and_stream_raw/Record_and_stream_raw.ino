/*
  Stream the microphone audio to serial port
  The file is RAW

  Precedure to extract audio: 
  NOTE: The baud rate is different from the codec example, due to the raw data 
  requiring a faster bit rate to avoid audio cut off.

  Setup the serial port as raw, for example with:
     stty -F /dev/ttyACM0 460800 raw
  Dump the data:
     cat /dev/ttyACM0 > test.g722
  If issues extracting try:
     stty -f /dev/cu.usbmodem* speed 460800 | cat /dev/cu.usbmodem* >test.pcm

  NOTE: must import into Audacity as raw data
  settings: encoding-> signed 16 bit / Byte order-> little-endian /
  channels-> single / sample rate-> 16 kHz

  NOTE: The recording may come out noisy or unwieldy due to the extraction 
  starting 1 bit late/early. 
  To fix this try:
     dd if=test.pcm of=test_fix.pcm bs=1 skip=1
*/

#include "Arduino.h"
#include "NDP.h"

#undef abs
#define USE_INT24_FROM_INT

uint8_t data[2048];
int n;

void ledGreenOn() {
  nicla::leds.begin();
  nicla::leds.setColor(green);
  delay(200);
  nicla::leds.setColor(off);
  nicla::leds.end();
}

void setup() {

  Serial.begin(460800);
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  NDP.onEvent(ledGreenOn);

  Serial.println("Loading synpackages");
  NDP.begin("mcu_fw_120_v105.synpkg");
  NDP.load("dsp_firmware_v105.synpkg");
  NDP.load("alexa_model334_ndp120_v105.synpkg");
  Serial.println("packages loaded");
  NDP.turnOnMicrophone();
  NDP.extractStart();
}

void loop() {
  unsigned int len = 0;
  int s;

  NDP.waitForAudio();
  for(;;) {
    s = NDP.extractData(data, &len);
    if (s) {
      break;
    }
    if (len) {
      Serial.write(data, len);
    }
  }
}