//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: FilterMovAvg
//---------------------------------------------------------------------------
/*
This class is part of the PetBot (PB) program. It is a moving average filter
with a window size defined using the constructor. The window size must be an
integer less than 255 and much smaller values are recommended to avoid memory
issues.

Author: Lloyd Fletcher
*/
#ifndef FILTERMOVAVG_H
#define FILTERMOVAVG_H

#include <Arduino.h>
#include "PB3DTimer.h"

class FilterMovAvg{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR/DESTRUCTOR
  FilterMovAvg();
  FilterMovAvg(uint8_t inWin);
  FilterMovAvg(uint8_t inWin, uint16_t inUpdateTime);
  ~FilterMovAvg();

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // FILTER
  double filter(double inData);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  void reset(float inVal);

  uint8_t get_window(){return _window;}
  double get_current_value(){return _curr_filtered;}

private:
  Timer _filter_timer;
  uint16_t _update_time = 1; // Default update time is 1ms (1kHz)
  uint8_t _window = 5;      // Default window is 5 data points
  uint8_t _currIndex = 0;
  double _curr_filtered = 0.0;
  double _dataSum = 0.0;
  double* _dataArray;
};
#endif // FILTERMOVAVG_H
