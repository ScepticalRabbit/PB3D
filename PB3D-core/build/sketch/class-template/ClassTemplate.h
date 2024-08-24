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
#include "PB3DTimer.h"


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
    bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Collision* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
   bool _enabled = true;
  bool _start_flag = true;

};
#endif
