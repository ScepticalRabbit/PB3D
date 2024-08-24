#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveBasic.h"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef CLASSTEMP_H
#define CLASSTEMP_H

#include <Arduino.h>

//---------------------------------------------------------------------------
// CLASS TEMPLATE: 
//---------------------------------------------------------------------------
class MoveBasic{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  //---------------------------------------------------------------------------
  MoveBasic();

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  //---------------------------------------------------------------------------
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  //---------------------------------------------------------------------------
  void update();

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree 
  //---------------------------------------------------------------------------
  void doSomething();

  //---------------------------------------------------------------------------
  // Get, set and reset
  //---------------------------------------------------------------------------
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  //---------------------------------------------------------------------------
  bool _isEnabled = true;
  bool _startFlag = true;

};
#endif 
