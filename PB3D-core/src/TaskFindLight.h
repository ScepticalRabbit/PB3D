//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKFINDLIGHT_H
#define TASKFINDLIGHT_H

#include <Wire.h>
#include <Adafruit_VEML7700.h>

#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"
#include "PatSensor.h"


class TaskFindLight{
public:
  TaskFindLight(MoodManager* mood, TaskManager* task, MoveManager* move,
                Speaker* speaker, PatSensor* pat_sens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // FINDLIGHT - called during the main during decision tree
  void find_light();
  void find_dark();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset_gradient();
  bool get_enabled_flag(){return _enabled;}
  float getLuxLeft(){return _luxLeft;}
  float getLuxRight(){return _luxRight;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _findLux(bool seekLightFlag);
  void _tcaSelect(uint8_t index);

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  PatSensor* _patSensObj = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

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
