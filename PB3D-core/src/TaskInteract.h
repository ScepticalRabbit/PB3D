//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskInteract
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

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

  bool getEnabledFlag(){return _isEnabled;}
  uint16_t getTimeOut(){return _patTimeOut;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

  
private:
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  TaskDance* _taskDanceObj = NULL;
  Speaker* _speakerObj = NULL;
  PatSensor* _patSensObj = NULL; 

  // TASK - INTERACT - Enabled Flag
  bool _isEnabled = true;

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
