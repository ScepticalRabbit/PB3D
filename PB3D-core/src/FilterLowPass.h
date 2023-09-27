//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: FilterLowPass
//---------------------------------------------------------------------------
/*
This class is part of the PetBot (PB) program. It is a 1st order low pass 
filter. 

Author: Lloyd Fletcher
*/
#ifndef FILTERLOWPASS_H
#define FILTERLOWPASS_H

#include <Arduino.h>
#include "Timer.h"

class FilterLowPass{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTORS
  FilterLowPass();
  FilterLowPass(double inAlpha);
  FilterLowPass(double inAlpha, uint16_t inUpdateTime);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // FILTER
  double filter(double inData);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void setAlpha(double inAlpha){_alpha = inAlpha;}
  double getAlpha(){return _alpha;}
  double getCurrVal(){return _currFiltered;}
  void reset(){_prevFiltered = 0.0;}
  void reset(double inVal){_prevFiltered = inVal;}

private:
  Timer _filtTimer;
  uint16_t _updateTime = 1; // Default update time is 1kHz
  double _alpha = 0.1;
  double _prevFiltered = 0.0;
  double _currFiltered = 0.0;
};
#endif // FILTERLOWPASS_H
