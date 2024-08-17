#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveBasic.h"
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

class MoveBasic{
public:
  MoveBasic();

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
  void doSomething();

  //---------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _is_enabled;}
  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  //---------------------------------------------------------------------------
  bool _is_enabled = true;
  bool _start_flag = true;

};
#endif
