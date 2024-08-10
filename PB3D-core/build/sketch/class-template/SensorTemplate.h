#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/class-template/SensorTemplate.h"
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

//---------------------------------------------------------------------------
// CLASS:
//---------------------------------------------------------------------------
class SensorTemp{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  SensorTemp();

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update();

  //---------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _is_enabled;}
  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _is_enabled = true;
  bool _start_flag = true;

};
#endif
