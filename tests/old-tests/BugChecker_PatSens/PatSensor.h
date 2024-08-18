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
#include "Timer.h"

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
    _disableButtonsTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    // SENSOR: Check for button press
    // 0x01 = b1, 0x02 = b2, 0x03 = b1+b2
    u8 valueButton = 0;
    _touchSens.get_touch_button_value(&valueButton);
    if(~_patFlag && _disableButtonsTimer.finished()){
      if((valueButton & 0x01)||(valueButton & 0x02)){
        _patFlag = true;
        _buttonFlag = true;
      }
      else{
        _buttonFlag = false;
      }
    }
  }

  //---------------------------------------------------------------------------
  // ACCEPT PATS - called during the main during decision tree
  void acceptPats(){
    if(!_enabled){return;}

    // Slider value, left=100, right=0
    u8 valueSlider = 0;
    _touchSens.get_touch_slider_value(&valueSlider);

    // See if we need to update the slider tolerance if it is in range
    if((valueSlider<=(_sensPatThres+_sensPatTol))&&(valueSlider>=(_sensPatThres-_sensPatTol))){
      _sensPatThres = _sensPatThres-_sensPatInc;
      if((_sensPatThres-_sensPatTol)<=0){
        // Reset the 'pat' state
        _sensPatThres = 100-_sensPatTol;
        _sensPatCount = _sensPatCount+1;
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}
  bool getPatFlag(){return _patFlag;}
  bool getButtonFlag(){return _buttonFlag;}
  uint8_t getPatCount(){return _sensPatCount;}

  bool getPatFinished(){
    return (_sensPatCount >= _sensPatCountThres);
  }

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}
  void setPatCountThres(uint8_t inCount){_sensPatCountThres = inCount;}

  //---------------------------------------------------------------------------
  // RESET FUNCTION
  void reset(){
    _patFlag = false;   // Reset the pat flag
    _sensPatCount = 0;  // Reset the pat counter
    // Disable buttons once pat is over
    _disableButtonsTimer.start(_disableButtonsTime);
  }

private:
  //---------------------------------------------------------------------------
  // TOUCH SENSOR
  CY8C _touchSens = CY8C();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  bool _patFlag = false;
  bool _buttonFlag = false;
  bool _patComplete = false;

  uint8_t _sensPatCountThres = 3;
  uint8_t _sensPatCount = 0;
  int16_t _sensPatTol = 10;
  int16_t _sensPatInc = 10;
  int16_t _sensPatThres = 100-_sensPatTol;

  // Timer to disable buttons after successful pat
  uint16_t _disableButtonsTime = 5000;
  Timer _disableButtonsTimer = Timer();

};
#endif // CLASS PATSENSOR
