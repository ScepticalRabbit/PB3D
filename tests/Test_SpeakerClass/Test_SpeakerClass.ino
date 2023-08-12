#include <Wire.h>
#include "Speaker.h"
#include "Timer.h"

uint32_t testTime = 4000;
Timer testTimer = Timer();

uint8_t soundType = SPEAKER_OFF;
Speaker speakerObj = Speaker();

void setup() {
  Serial.begin(115200); 
  
  speakerObj.begin();
  speakerObj.setSoundParams(NOTE_G3,NOTE_B2,NOTE_G3,NOTE_B2,
                            1000,200,1000,200);
                            
  testTimer.start(testTime);
}

void loop() {
  speakerObj.update();

  
  if(testTimer.finished()){
    Serial.println("Test timer finished.");
    testTimer.start(testTime);
    speakerObj.setSoundCode(soundType);
    speakerObj.reset();
  }
}
