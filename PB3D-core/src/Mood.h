//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: MOOD
//---------------------------------------------------------------------------
/*
The mood class is part of the PetBot (PB) program. It controls what mood PB3D 
is in. The mood is indicated using the inner LEDs and the task is indicated 
with the outer LEDs.

Author: Lloyd Fletcher
Date Created: 28th Aug. 2021
Date Edited:  28th Aug. 2021 
*/
#ifndef MOOD_H
#define MOOD_H

#include <Arduino.h>
//#include <Adafruit_NeoPixel.h> // RGB LEDs
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "Timer.h"

// Mood LEDs
#define MOOD_PIN 6
#define MOOD_NUMPIX 4

// MOOD Codes - Save some memory
#define MOOD_TEST -1
#define MOOD_NEUTRAL 0
#define MOOD_HAPPY 1
#define MOOD_SAD 2
#define MOOD_ANGRY 3
#define MOOD_SCARED 4

class Mood{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Mood(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){
    _moodLEDs = RGBLEDs;
  }
  
  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    // Generate a probability, start the mood timer and set the mood
    _moodPc = random(0,100); // NOTE: random num between (min,max-1)
    setMood(_moodPc);
    _moodTimer.start(_moodDuration);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update(){
    if(_moodTimer.finished()){
      // Randomly update mood score before generating new mood
      int8_t prob = random(0,100);
      if(prob <= 10){decMoodScore();}
      else if(prob >= 90){incMoodScore();}

      // Generate a new mood probability
      _moodPc = random(0,100)+_moodScore; // NOTE: random num between (min,max-1)
      _moodPc = constrain(_moodPc,0,100);

      // Determine where on the happy/sad axis
      if((_moodPc >= 0) && (_moodPc < _moodProbVec[0])){ // NEGATIVE MOOD
        _moodNegPc = random(0,100)+_moodScore;
        
        if((_moodNegPc >= 0) && (_moodNegPc <= _moodProbVec[0])){
          setMood(MOOD_SAD);
        }
        else if((_moodPc >= _moodProbVec[1]) && (_moodPc <= _moodProbVec[2])){
          setMood(MOOD_ANGRY);  
        }
        else if((_moodPc >= _moodProbVec[2]) && (_moodPc <= _moodProbVec[3])){
          setMood(MOOD_SCARED);
        }
        else{
          setMood(MOOD_SAD);
        }
      }
      else if((_moodPc >= _moodProbVec[0]) && (_moodPc <= _moodProbVec[1])){ // NEUTRAL MOOD
        setMood(MOOD_NEUTRAL);
      }
      else if((_moodPc >= _moodProbVec[1]) && (_moodPc <= _moodProbVec[2])){ // POSITIVE MOOD
        setMood(MOOD_HAPPY);
      }
      else{ // DEFAULT NEUTRAL
        setMood(MOOD_NEUTRAL);
      }
    }
  }

  //---------------------------------------------------------------------------
  // Get, set and reset
  void setMood(int16_t moodIn){
    if(moodIn != _moodCode){
      _setMood(moodIn);
      
      _moodNewFlag = true;
      _moodDuration = random(_moodMinDuration,_moodMaxDuration);
      _moodTimer.start(_moodDuration);
    }
  }
  
  void resetMood(){_setMood(_moodCode);}
  int8_t getMood(){return _moodCode;}
  int8_t getPowerDiff(){return _moodPowerDiffVec[_moodCode];}
  float getSpeedFact(){return _moodSpeedFactVec[_moodCode];}
  uint8_t getMoodCount(){return _moodCount;}
  bool getNewMoodFlag(){return _moodNewFlag;}
  void setNewMoodFlag(bool inFlag){_moodNewFlag = inFlag;}

  //---------------------------------------------------------------------------
  // MOOD SCORE FUNCTIONS
  int8_t getMoodScore(){return _moodScore;}
  void setMoodScore(int8_t inScore){_moodScore = constrain(inScore,_moodScoreMin,_moodScoreMax);}
  void resetMoodScore(){_moodScore = 0;}

  void modMoodScore(int8_t modifier){
    int16_t tempScore = _moodScore+modifier;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
  }

  void incMoodScore(){
    int16_t tempScore = _moodScore+_moodScoreInc;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
  }
  
  void decMoodScore(){
    int16_t tempScore = _moodScore-_moodScoreInc;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
  }

  //---------------------------------------------------------------------------
  // MOOD LED FUNCTIONS
  // Functions for setting mood LEDs
  
  void moodLEDNeutral(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->show();
  }
  
  void moodLEDHappy(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->show();
  }
  
  void moodLEDAngry(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->show();
  }
  
  void moodLEDSad(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->show();
  }
  
  void moodLEDScared(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->show();
  }
  
  void _moodLEDCurious(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->show();
  }
  
  void moodLEDTest(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->show();
  }
  
  void moodLEDOff(){
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->show();
  }
  
private:
  //---------------------------------------------------------------------------
  // MOOD FUNCTIONS
  void _setMood(int8_t moodIn){
    if(moodIn == MOOD_NEUTRAL){
      _moodCode = MOOD_NEUTRAL;
      moodLEDNeutral();
    }
    else if(moodIn == MOOD_HAPPY){
      _moodCode = MOOD_HAPPY;
      moodLEDHappy();
    }
    else if(moodIn == MOOD_SAD){
      _moodCode = MOOD_SAD;
      moodLEDSad();
    }
    else if(moodIn == MOOD_ANGRY){
      _moodCode = MOOD_ANGRY;
      moodLEDAngry();
    }
    else if(moodIn == MOOD_SCARED){
      _moodCode = MOOD_SCARED;
      moodLEDScared();
    }
    else if(moodIn == MOOD_TEST){
      _moodCode = MOOD_TEST;
      moodLEDTest();
    }
    else{
      _moodCode = MOOD_NEUTRAL;
      moodLEDNeutral();
    }
  }
  
  //---------------------------------------------------------------------------=
  // MOOD VARIABLES
  // Moods: [neutral,happy,sad,angry,scared]
  // Mood changes task probabilities
  int8_t _moodCode = MOOD_NEUTRAL;
  int16_t _moodPc = 0, _moodNegPc = 0;
  uint8_t _moodCount = 5;
  bool  _moodNewFlag = true;

  // Mood score variables
  int8_t _moodScore = 0, _moodScoreInc = 10;
  int8_t _moodScoreMin = -50, _moodScoreMax = 50;

  Timer _moodTimer;
  uint32_t _moodDuration = 12000;
  uint32_t _moodMinDuration = 10000;
  uint32_t _moodMaxDuration = 40000;

  // Moods: [neutral,happy,sad,angry,scared]
  // Mood axis = [negative,neutral,positive]
  // Negative moods = [sad,angry,scared]
  const int8_t _moodProbVec[5] =   {40,70,80,90,100};
  const int8_t _moodProbPNAxisVec[3] = {25,75,100};
  const int8_t _moodProbNegVec[3] = {33,66,100};
  const int8_t _moodPowerDiffVec[5] =  {0,10,-30,20,-10};
  //const float _moodSpeedDiffVec[5] = {0.0,10.0,-30.0,20.0,-10.0};
  const float _moodSpeedFactVec[5] = {1.0,1.1,0.7,1.2,0.9};

  // Pointer to the mood expression LEDs
  Adafruit_NeoPixel_ZeroDMA* _moodLEDs;
};
#endif // MOOD_H
