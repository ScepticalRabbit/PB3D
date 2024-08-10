#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Encoder.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "Timer.h"
#include "FilterMovAvg.h"
#include "FilterLowPass.h"

class Encoder{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR
  Encoder(int8_t pinA, int8_t pinB);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  // Update for the right encoder '!='
  void updateNEQ();

  // Update for the left encoder '=='
  void updateEQ();

  //-------------------------------------------------------------------------
  // SPEED CALCULATION FUNCTIONS
  void updateSpeed();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void resetFilter();
  void resetFilter(float inVal);

  int32_t getCount(){return _encCountCurr;}
  void setCount(int32_t inCount){_encCountCurr = inCount;}
  char getDir(){return _encDirCode;}
  double getMMPerCount(){return _mmPerCount;}
  double getRawSpeedMMPS(){return _rawSpeedMMPS;}
  double getSmoothSpeedMMPS(){return _smoothSpeedMMPS;}
  double getRawSpeedCPS(){return _rawSpeedCPS;}
  double getSmoothSpeedCPS(){return _smoothSpeedCPS;}
  uint16_t getSpeedUpdateTime(){return _speedUpdateTime;};

private:
  // CORE VARIABLES - Used by all types of encoder
  volatile int32_t _encCountCurr;
  char _encDirCode = 'F';
  int8_t _pinA = -1;
  int8_t _pinB = -1;

  // SPEED VARIABLES - Used by robot wheel encoders
  Timer _speedTimer = Timer();

  // Characteristics of the motor/gears/wheel/encoder
  double _wheelDiam = 60.0;
  double _wheelCirc = _wheelDiam*PI;
  double _gearRatio = 50.0, _encCountsPerRev = 12.0;
  double _countsPerRev = _gearRatio*_encCountsPerRev;
  double _mmPerCount = _wheelCirc/_countsPerRev;

  uint16_t _speedUpdateTime = 10;      // Default update time of 100Hz
  uint16_t _speedFiltUpdateTime = 5;  // Must be less than speed update time.
  int32_t _encCountPrevForSpeed = 0;

  uint8_t _speedFiltWin = 10;
  float _speedFiltAlpha = 0.1; // NOTE: lower alpha for more smoothing!
  // NOTE: filter is called within speed update loop so set update time to
  // be less than the speed one so we update the filter at the same rate
  // as the speed
  //FilterMovAvg _speedFilt = FilterMovAvg(_speedFiltWin,_speedUpdateTime/2);
  FilterLowPass _speedFiltMMPS = FilterLowPass(_speedFiltAlpha,_speedFiltUpdateTime);
  double _rawSpeedMMPS = 0.0, _smoothSpeedMMPS = 0.0;
  FilterLowPass _speedFiltCPS = FilterLowPass(_speedFiltAlpha,_speedFiltUpdateTime);
  double _rawSpeedCPS = 0.0, _smoothSpeedCPS = 0.0;
};
#endif // ENCODER_H
