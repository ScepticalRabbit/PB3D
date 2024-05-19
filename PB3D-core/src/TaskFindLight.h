//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskFindLight
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#ifndef TASKFINDLIGHT_H
#define TASKFINDLIGHT_H

#include <Wire.h> // I2C
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"
#include "PatSensor.h"

// TASKFINDLIGHT: specific defines/includes
#define TCAADDR 0x70
#define LIGHTSENS_L 1
#define LIGHTSENS_R 0 
#include "Adafruit_VEML7700.h"

class TaskFindLight{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskFindLight(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, 
                Speaker* inSpeaker, PatSensor* inPatSens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // FINDLIGHT - called during the main during decision tree
  void findLight();
  void findDark();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void resetGrad();
  bool getEnabledFlag(){return _isEnabled;}
  float getLuxLeft(){return _luxLeft;}
  float getLuxRight(){return _luxRight;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _findLux(bool seekLightFlag);
  void _tcaSelect(uint8_t index);

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;
  PatSensor* _patSensObj = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

  Adafruit_VEML7700 _lightSensL = Adafruit_VEML7700();
  Adafruit_VEML7700 _lightSensR = Adafruit_VEML7700();

  uint16_t _senseUpdateTime = 400;
  Timer _senseTimer = Timer();
  float _luxLeft = 0.0, _luxRight = 0.0;
  float _luxAvg = 0.0, _luxDiff = 0.0;
  float _luxPcThresMid = 0.2, _luxThresMid = 0.0;
  float _luxPcThresStr = 0.5, _luxThresStr = 0.0;
  float _luxThresStop = 20.0;

  uint16_t _gradUpdateTime = 5000;
  Timer _gradTimer = Timer();
  float _luxLRAvgT0 = 0.0, _luxLRAvgT1 = 0.0;
  float _luxTAvg = 0.0;
  float _luxGrad = 0.0;
  float _luxGradPcThres = 0.2, _luxGradThres = 0.0;
  bool _gradMoveFlag = false;
  Timer _gradMoveTimeout = Timer();
  uint16_t _gradMoveTimeoutTime = 2000;
};
#endif // End TASKFINDLIGHT
