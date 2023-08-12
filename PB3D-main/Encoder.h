//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - Encoder
//---------------------------------------------------------------------------
/*
The encoder class is part of the PetBot (PB) program.

Author: Lloyd Fletcher
Date Created: 29th Aug. 2021
Date Edited:  29th Aug. 2021 
*/

#ifndef ENCODER_H
#define ENCODER_H

#include "Timer.h"
#include "FilterMovAvg.h" 
#include "FilterLowPass.h"

class Encoder{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR
  Encoder(int8_t pinA, int8_t pinB){    
    _pinA = pinA;
    _pinB = pinB;
    _encCountCurr = 0;
  }
  
  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);
    _speedTimer.start(0);
    _speedFiltMMPS.begin();
    _speedFiltCPS.begin();
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  
  // Update for the right encoder '!='
  void updateNEQ(){
    if(digitalRead(_pinA) != digitalRead(_pinB)){
      _encCountCurr++;
      _encDirCode = 'F';
    }
    else{
      _encCountCurr--;
      _encDirCode = 'B';
    } 
  }
  
  // Update for the left encoder '=='
  void updateEQ(){
    if(digitalRead(_pinA) == digitalRead(_pinB)){
      _encCountCurr++;
      _encDirCode = 'F';
    }
    else{
      _encCountCurr--;
      _encDirCode = 'B';
    } 
  }

  int32_t getCount(){
    return _encCountCurr;
  }

  void setCount(int32_t inCount){
    _encCountCurr = inCount;
  }

  char getDir(){
    return _encDirCode;
  }

  //-------------------------------------------------------------------------
  // SPEED CALCULATION FUNCTIONS
  void updateSpeed(){
    if(_speedTimer.finished()){
      // Get the current time and restart the timer
      double timeIntS = double(_speedTimer.getTime())/1000.0;
      _speedTimer.start(_speedUpdateTime);
      
      // Calculate distance travelled using the encoder count difference
      double distMM = double(_encCountCurr-_encCountPrevForSpeed)*_mmPerCount;

      // Calculate speed in 'counts/second'
      _rawSpeedCPS = double(_encCountCurr-_encCountPrevForSpeed)/timeIntS;
      // Smooth the raw speed using the filter
      _smoothSpeedCPS = _speedFiltCPS.filter(_rawSpeedCPS); 
      
      // Calculate raw and smoothed speed
      _rawSpeedMMPS = distMM/timeIntS;
      // Smooth the raw speed using the filter
      _smoothSpeedMMPS = _speedFiltMMPS.filter(_rawSpeedMMPS); 

      // Store the encoder count for the next round of calculations
      _encCountPrevForSpeed = _encCountCurr;
    }
  }

  void resetFilter(){
    _speedFiltMMPS.reset();
    _speedFiltCPS.reset();
  }

  void resetFilter(float inVal){
    _speedFiltMMPS.reset(inVal);
    _speedFiltCPS.reset(inVal);
  }

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
  int8_t _pinA;
  int8_t _pinB;

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
