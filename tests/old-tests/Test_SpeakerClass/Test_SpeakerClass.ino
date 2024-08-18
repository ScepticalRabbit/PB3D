#include <Wire.h>
#include "Speaker.h"
#include "Timer.h"

uint32_t testTime = 4000;
Timer testTimer = Timer();

uint8_t soundType = SPEAKER_OFF;
Speaker speaker = Speaker();

void setup() {
  Serial.begin(115200);

  speaker.begin();
  speaker.setSoundParams(NOTE_G3,NOTE_B2,NOTE_G3,NOTE_B2,
                            1000,200,1000,200);

  testTimer.start(testTime);
}

void loop() {
  speaker.update();


  if(testTimer.finished()){
    Serial.println("Test timer finished.");
    testTimer.start(testTime);
    speaker.setSoundCode(soundType);
    speaker.reset();
  }
}
