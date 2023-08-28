//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - Filter Low Pass
//---------------------------------------------------------------------------
/*
This class is part of the PetBot (PB) program. It is a 1st order low pass 
filter. The filter

Author: Lloyd Fletcher
Date Created: 5th Sep. 2021
Date Edited:  5th Sep. 2021 
*/
#ifndef FILTERLOWPASS_H
#define FILTERLOWPASS_H

#include "Timer.h"

class FilterLowPass{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTORS
  FilterLowPass(){
  }

  FilterLowPass(double inAlpha){
    _alpha = inAlpha;
  }

  FilterLowPass(double inAlpha, uint16_t inUpdateTime){
    _alpha = inAlpha;
    _updateTime = inUpdateTime;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    _filtTimer.start(0);
  }

  double filter(double inData){
    if(_filtTimer.finished()){
      _currFiltered = _alpha*inData + (1-_alpha)*_prevFiltered;
      _prevFiltered = _currFiltered;
      _filtTimer.start(_updateTime);
    }
    return _currFiltered;
  }

  void setAlpha(double inAlpha){
    _alpha = inAlpha;
  }

  double getAlpha(){
    return _alpha;
  }

  double getCurrVal(){
    return _currFiltered;
  }

  void reset(){
    _prevFiltered = 0.0;
  }

  void reset(float inVal){
    _prevFiltered = inVal;
  }

private:
  Timer _filtTimer;
  uint16_t _updateTime = 1; // Default update time is 1kHz
  double _alpha = 0.1;
  double _prevFiltered = 0.0;
  double _currFiltered = 0.0;
};
#endif // FILTERLOWPASS_H
