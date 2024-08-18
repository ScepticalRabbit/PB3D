//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKINTERACT_H
#define TASKINTERACT_H

#include <Wire.h> // I2C
#include <Seeed_CY8C401XX.h> // Capcitive Touch Sensor
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "TaskDance.h"
#include "Speaker.h"
#include "Timer.h"
#include "PatSensor.h"

class TaskInteract{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskInteract(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
               Speaker* inSpeaker, TaskDance* inDance, PatSensor* inPatSens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // INTERACT - called during the main during decision tree
  void interact();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void setStartInteractFlag(bool inFlag);

  bool get_enabled_flag(){return _enabled;}
  uint16_t getTimeOut(){return _patTimeOut;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}


private:
  // MAIN OBJECT POINTERS
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  TaskDance* _taskDanceObj = NULL;
  Speaker* _speakerObj = NULL;
  PatSensor* _patSensObj = NULL;

  // TASK - INTERACT - Enabled Flag
  bool _enabled = true;

  // TASK - INTERACT Variables
  bool _interactStartFlag = true;

  // 'Ask' update variables
  bool _askFlag = false;
  uint16_t _askSqueakInterval = 7000;
  Timer _askSqueakTimer = Timer();
  uint16_t _askWiggleLeftDur = 500;
  uint16_t _askWiggleRightDur = 500;
  uint16_t _askWiggleDuration = 2*(_askWiggleLeftDur+_askWiggleRightDur);
  Timer _askWiggleTimer = Timer();


  uint16_t _patTimeOut = 30000;
  Timer _patTimeOutTimer = Timer();
};
#endif // TASKINTERACT
