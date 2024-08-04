#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskRest.h"
//---------------------------------------------------------------------------
// PET BOT - PB3! 
// CLASS: TaskRest
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef TASKREST_H
#define TASKREST_H

#include <Arduino.h>
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

class TaskRest{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskRest(MoodManager* inMood, TaskManager* inTask, 
            MoveManager* inMove, Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // REST
  void rest();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  
private:
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;

  // Timers
  Timer _timerObj = Timer();

  // SUBTASK - REST Variables
  // Use to pulse LEDs during rest task
  uint8_t _restUpdateTime = 7;
  bool _restLEDIncrease = true;
  uint8_t _restLEDMax = 254;
  uint8_t _restLEDMin = 0;
  uint8_t _restLEDVal = 254;
};
#endif // TASKREST
