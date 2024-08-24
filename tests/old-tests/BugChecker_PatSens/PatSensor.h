//---------------------------------------------------------------------------
// PET BOT - PB3D!
// CLASS - PatSensor.h
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 20th December 2021
Date Edited:  20th December 2021
*/

#ifndef PATSENSOR_H
#define PATSENSOR_H

#include <Wire.h> // I2C
#include <Seeed_CY8C401XX.h> // Capacitive Touch Sensor
#include "PB3DTimer.h"

#define ADDR_TOUCHSENS 0x08

class PatSensor{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  PatSensor(){
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // Ping the touch sensor to see if it is connected
    Wire.beginTransmission(ADDR_TOUCHSENS);
    if(Wire.endTransmission() != 0){
      Serial.println(F("PATSENSOR: Failed to init touch sensor."));
      _enabled = false;
    }
    else{
      _enabled = true;
    }
    // Start Timers
    _disable_buttons_timer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    // SENSOR: Check for button press
    // 0x01 = b1, 0x02 = b2, 0x03 = b1+b2
    u8 valueButton = 0;
    _touch_sens.get_touch_button_value(&valueButton);
    if(~_pat_flag && _disable_buttons_timer.finished()){
      if((valueButton & 0x01)||(valueButton & 0x02)){
        _pat_flag = true;
        _buttonFlag = true;
      }
      else{
        _buttonFlag = false;
      }
    }
  }

  //---------------------------------------------------------------------------
  // ACCEPT PATS - called during the main during decision tree
  void accept_pats(){
    if(!_enabled){return;}

    // Slider value, left=100, right=0
    u8 valueSlider = 0;
    _touch_sens.get_touch_slider_value(&valueSlider);

    // See if we need to update the slider tolerance if it is in range
    if((valueSlider<=(_sens_pat_thres+_sens_pat_tol))&&(valueSlider>=(_sens_pat_thres-_sens_pat_tol))){
      _sens_pat_thres = _sens_pat_thres-_sens_pat_inc;
      if((_sens_pat_thres-_sens_pat_tol)<=0){
        // Reset the 'pat' state
        _sens_pat_thres = 100-_sens_pat_tol;
        _sens_pat_count = _sens_pat_count+1;
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}
  bool get_pat_flag(){return _pat_flag;}
  bool getButtonFlag(){return _buttonFlag;}
  uint8_t get_pat_count(){return _sens_pat_count;}

  bool get_pat_finished(){
    return (_sens_pat_count >= _sens_pat_count_thres);
  }

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}
  void set_pat_count_thres(uint8_t inCount){_sens_pat_count_thres = inCount;}

  //---------------------------------------------------------------------------
  // RESET FUNCTION
  void reset(){
    _pat_flag = false;   // Reset the pat flag
    _sens_pat_count = 0;  // Reset the pat counter
    // Disable buttons once pat is over
    _disable_buttons_timer.start(_disable_buttons_time);
  }

private:
  //---------------------------------------------------------------------------
  // TOUCH SENSOR
  CY8C _touch_sens = CY8C();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  bool _pat_flag = false;
  bool _buttonFlag = false;
  bool _pat_complete = false;

  uint8_t _sens_pat_count_thres = 3;
  uint8_t _sens_pat_count = 0;
  int16_t _sens_pat_tol = 10;
  int16_t _sens_pat_inc = 10;
  int16_t _sens_pat_thres = 100-_sens_pat_tol;

  // Timer to disable buttons after successful pat
  uint16_t _disable_buttons_time = 5000;
  Timer _disable_buttons_timer = Timer();

};
#endif // CLASS PATSENSOR
