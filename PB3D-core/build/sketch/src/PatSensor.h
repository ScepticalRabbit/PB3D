#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/PatSensor.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PATSENSOR_H
#define PATSENSOR_H

#include <Arduino.h>
#include <Wire.h> // I2C
#include <Seeed_CY8C401XX.h> // Capacitive Touch Sensor
#include "Timer.h"

#define ADDR_TOUCHSENS 0x08

class PatSensor{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  PatSensor(){}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // ACCEPT PATS - called during the main during decision tree
  void acceptPats();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  void genPatCountThres();

  bool get_enabled_flag(){return _is_enabled;}
  bool getPatFlag(){return _patFlag;}
  bool getButtonOneFlag(){return _buttonOneFlag;}
  bool getButtonTwoFlag(){return _buttonTwoFlag;}
  uint8_t getPatCount(){return _sensPatCount;}
  bool getPatFinished(){return (_sensPatCount >= _sensPatCountThres);}

  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}
  void setPatFlag(bool inFlag){_patFlag = inFlag;}
  void setPatCountThres(uint8_t inCount){_sensPatCountThres = inCount;}
  void setButtonsEnabled(bool inFlag){_buttonsEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // TOUCH SENSOR
  CY8C _touchSens = CY8C();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _is_enabled = true;

  bool _patFlag = false;
  bool _patComplete = false;

  bool _buttonOneFlag = false;
  bool _buttonTwoFlag = false;
  bool _buttonsEnabled = true;

  uint8_t _sensPatCountThres = 3;
  uint8_t _sensPatCountThresMin = 2, _sensPatCountThresMax = 5;
  uint8_t _sensPatCount = 0;
  int16_t _sensPatTol = 10;
  int16_t _sensPatInc = 10;
  int16_t _sensPatThres = 100-_sensPatTol;

  // Timer to disable buttons after successful pat
  uint16_t _disableButtonsTime = 5000;
  Timer _disableButtonsTimer = Timer();

};
#endif // CLASS PATSENSOR
