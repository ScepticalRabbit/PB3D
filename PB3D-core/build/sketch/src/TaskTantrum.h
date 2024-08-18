#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskTantrum.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#ifndef TASKTANTRUM_H
#define TASKTANTRUM_H

#include <Arduino.h>
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

class TaskTantrum{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskTantrum(MoodManager* inMood, TaskManager* inTask,
              MoveManager* inMove, Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // TANTRUM
  void haveTantrum();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void setStartTantrumFlag();

  uint8_t getThreshold(){return _tantrumThreshold;}
  uint16_t getDuration(){return _tantrumDuration;}
  bool getCompleteFlag(){return _tantrumComplete;}

private:
  // MAIN OBJECT POINTERS
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  // Timers
  Timer _timerObj1 = Timer();
  Timer _timerObj2 = Timer();

  // SUBTASK - TANTRUM Variables
  bool _startTantrumFlag = false;
  bool _tantrumComplete = false;
  uint16_t _tantrumThreshold = 31;
  uint16_t _tantrumFBDuration = 400;
  uint8_t _tantrumFBNum = 5;
  uint16_t _tantrumDuration = 2*_tantrumFBNum*_tantrumFBDuration;

  bool _growlFlag = true;
  uint16_t _tantrumGrowlDuration = 2*_tantrumFBDuration;
  uint16_t _tantrumGrowlPause = 1*_tantrumFBDuration;
};
#endif // TASKTANTRUM
