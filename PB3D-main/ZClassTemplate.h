//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS - TEMPLATE
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 19th December 2021
Date Edited:  19th December 2021 
*/

#ifndef CLASSTEMP_H
#define CLASSTEMP_H

#include <Wire.h> // I2C
#include "Collision.h" 
#include "Task.h"
#include "Move.h"
#include "Timer.h"
#include "Speaker.h"

class ClassTemplate{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  ClassTemplate(Collision* inCollision, Mood* inMood, Task* inTask, Move* inMove, 
                Speaker* inSpeaker){
    _collisionObj = inCollision;
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){

  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
      _startFlag = true;
    }

  }

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
  void doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
      _startFlag = false;
    }

  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Collision* _collisionObj;
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
  Speaker* _speakerObj;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;
  bool _startFlag = true;

};
#endif //
