#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/class-template/ClassTemplate.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef CLASSTEMP_H
#define CLASSTEMP_H

#include <Arduino.h>
#include <Wire.h> // I2C
#include "CollisionManager.h"
#include "Task.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"


//---------------------------------------------------------------------------
// CLASS TEMPLATE:
//---------------------------------------------------------------------------
class ClassTemp{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
 ClassTemp(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
                Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update();

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
   void doSomething();

  //---------------------------------------------------------------------------
  // Get, set and reset
    bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Collision* _collisionObj = NULL;
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
   bool _isEnabled = true;
  bool _startFlag = true;

};
#endif
