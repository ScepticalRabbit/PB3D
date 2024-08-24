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
#include "PB3DTimer.h"

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
  void set_alpha(double inAlpha){_alpha = inAlpha;}
  double get_alpha(){return _alpha;}
  double get_current_value(){return _curr_filtered;}
  void reset(){_prev_filtered = 0.0;}
  void reset(double inVal){_prev_filtered = inVal;}

private:
  Timer _filter_timer;
  uint16_t _update_time = 1; // Default update time is 1kHz
  double _alpha = 0.1;
  double _prev_filtered = 0.0;
  double _curr_filtered = 0.0;
};
#endif // FILTERLOWPASS_H
