//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOODMANAGER_H
#define MOODMANAGER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

#include "PB3DConstants.h"
#include "Timer.h"

// Mood LEDs
#define MOOD_PIN 6
#define MOOD_NUMPIX 4

class MoodManager{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  MoodManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){_moodLEDs = RGBLEDs;}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // Get, set and reset
  //---------------------------------------------------------------------------
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}
  void reset_mood(){_setMood(_moodCode);}
  int8_t get_mood(){return _moodCode;}
  int8_t getPowerDiff(){return _moodPowerDiffVec[_moodCode];}
  float getSpeedFact(){return _moodSpeedFactVec[_moodCode];}
  uint8_t getMoodCount(){return _moodCount;}
  bool getNewMoodFlag(){return _moodNewFlag;}
  void setNewMoodFlag(bool inFlag){_moodNewFlag = inFlag;}

  void setMood(int16_t moodIn);

  //---------------------------------------------------------------------------
  // Mood Score Functions
  //---------------------------------------------------------------------------

  int8_t getMoodScore(){return _moodScore;}
  void setMoodScore(int8_t inScore){_moodScore = constrain(inScore,_moodScoreMin,_moodScoreMax);}
  void resetMoodScore(){_moodScore = 0;}

  void modMoodScore(int8_t modifier);
  void incMoodScore();
  void decMoodScore();

  //---------------------------------------------------------------------------
  // Mood LED Functions
  //---------------------------------------------------------------------------
  void moodLEDNeutral();
  void moodLEDHappy();
  void moodLEDAngry();
  void moodLEDSad();
  void moodLEDScared();
  void moodLEDCurious();
  void moodLEDTest();
  void moodLEDOff();

private:
  //---------------------------------------------------------------------------
  // Helper Functions
  //---------------------------------------------------------------------------
  void _setMood(int8_t moodIn);

  //---------------------------------------------------------------------------=
  // MOOD VARIABLES
  //---------------------------------------------------------------------------=
  // Moods: [neutral,happy,sad,angry,scared]
  // Mood changes task probabilities
  bool _enabled = true;
  EMoodCode _moodCode = MOOD_NEUTRAL;
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
  Adafruit_NeoPixel_ZeroDMA* _moodLEDs = NULL;
};
#endif // MOOD_H
