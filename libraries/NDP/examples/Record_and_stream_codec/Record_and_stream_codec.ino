/*
  Stream the microphone audio to serial port
  The file is compressed using G722 codec
  Prerequisite libraries:
    https://github.com/pschatzmann/arduino-libg722
    https://github.com/pschatzmann/arduino-audio-tools/
    v0.9.3(older version) is required

  Precedure to extract audio:
  Setup the serial port as raw, for example with
     stty -F /dev/ttyACM0 115200 raw
  Dump the data
     cat /dev/ttyACM0 > test.g722
  If issues extracting (2 steps above) try:
     stty -f /dev/cu.usbmodem* speed 115200 | cat /dev/cu.usbmodem* >test.g722
  Open Audacity
     audacity test.g722
*/

#include "Arduino.h"
#include "NDP.h"

#undef abs
#define USE_INT24_FROM_INT
#include "AudioTools.h"
#include "AudioCodecs/CodecG722.h"

G722Encoder encoder;

uint8_t data[2048];

void ledGreenOn() {
  nicla::leds.begin();
  nicla::leds.setColor(green);
  delay(200);
  nicla::leds.setColor(off);
  nicla::leds.end();
}

void setup() {

  Serial.begin(115200);
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  NDP.onEvent(ledGreenOn);

  AudioBaseInfo bi;
  bi.channels = 1;
  bi.sample_rate = 16000;

  encoder.setOptions(0);
  encoder.begin(bi);

  encoder.setOutputStream(Serial);

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
      encoder.write(data, len);
    }
  }
}