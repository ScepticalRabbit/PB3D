//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: PAUSE
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 12th December 2021
Date Edited:  12th December 2021 
*/

#ifndef TASKPAUSE_H
#define TASKPAUSE_H

#include <Wire.h> // I2C
#include "CollisionManager.h" 
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"

class TaskPause{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskPause(CollisionManager* inCollision, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _collisionObj = inCollision;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    _pauseTimer.start(0);
  }

  void update(){
    if(!_isEnabled){return;}
    
    if(_taskObj->getNewTaskFlag()){
      _pauseDur = random(_pauseDurMin,_pauseDurMax);
      _pauseTimer.start(_pauseDur);
    }
  }

  void pause(){
    if(!_isEnabled){return;}
    
    if(_pauseTimer.finished()){
      _taskObj->forceUpdate();
    }
    else{
      _taskObj->taskLEDPause(_pauseDur);
      _collisionObj->setEnabledFlag(false);
      _moveObj->stop();
    }
  }

  // GET/SET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  // MAIN OBJECT POINTERS
  CollisionManager* _collisionObj;
  MoodManager* _moodObj;
  TaskManager* _taskObj;
  MoveManager* _moveObj;
  Speaker* _speakerObj;

  // TASK - PAUSE
  bool _isEnabled = true;
  Timer _pauseTimer = Timer();
  uint16_t _pauseDur = 2000;
  uint16_t _pauseDurMin = 2000, _pauseDurMax = 4000; 
};
#endif // PAUSE
