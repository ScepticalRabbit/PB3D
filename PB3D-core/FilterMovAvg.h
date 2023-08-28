//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - Filter Low Pass
//---------------------------------------------------------------------------
/*
This class is part of the PetBot (PB) program. It is a moving average filter
with a window size defined using the constructor. The window size must be an
integer less than 255 and much smaller values are recommended to avoid memory
issues.

Author: Lloyd Fletcher
Date Created: 5th Sep. 2021
Date Edited:  5th Sep. 2021 
*/
#ifndef FILTERMOVAVG_H
#define FILTERMOVAVG_H

#include "Timer.h"

class FilterMovAvg{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR/DESTRUCTOR
  FilterMovAvg(){
    _dataArray = new double[_window];
    for(uint8_t ii=0 ; ii<_window ; ii++){
      _dataArray[ii] = 0.0;
    }
  }

  FilterMovAvg(uint8_t inWin){
    _window = inWin;
    _dataArray = new double[inWin];
    for(uint8_t ii=0 ; ii<inWin ; ii++){
      _dataArray[ii] = 0.0;
    }
  }

  FilterMovAvg(uint8_t inWin, uint16_t inUpdateTime){
    _window = inWin;
    _updateTime = inUpdateTime;
    _dataArray = new double[inWin];
    for(uint8_t ii=0 ; ii<inWin ; ii++){
      _dataArray[ii] = 0.0;
    }
  }

  ~FilterMovAvg(){
    delete []_dataArray;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    _filtTimer.start(0);
  }

  double filter(double inData){
    if(_filtTimer.finished()){
      // Remove the data point at the current position from the running total
      _dataSum = _dataSum - _dataArray[_currIndex];
      // Overwrite the data point at the current position
      _dataArray[_currIndex] = inData;
      // Add the current data point to the running total
      _dataSum = _dataSum + inData;
      // Increment the index and wrap around if needed
      _currIndex++;
      if(_currIndex >= _window){
        _currIndex = 0;
      }
      // The filtered value is the average over our window, return it
      _currFiltered = _dataSum / _window;

      _filtTimer.start(_updateTime);
    }
    return _currFiltered;
  }

  void reset(){
    for(uint8_t ii=0 ; ii<_window ; ii++){
      _dataArray[ii] = 0.0;
    }
  }

  void reset(float inVal){
    for(uint8_t ii=0 ; ii<_window ; ii++){
      _dataArray[ii] = inVal;
    }
  }

  uint8_t getWindow(){
    return _window;
  }

  double getCurrVal(){
    return _currFiltered;
  }

private:
  Timer _filtTimer; 
  uint16_t _updateTime = 1; // Default update time is 1ms (1kHz)
  uint8_t _window = 5;      // Default window is 5 data points
  uint8_t _currIndex = 0;
  double _currFiltered = 0.0;
  double _dataSum = 0.0;
  double* _dataArray;
};
#endif // FILTERMOVAVG_H
