//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: 
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

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
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;
  bool _startFlag = true;

};
#endif 
