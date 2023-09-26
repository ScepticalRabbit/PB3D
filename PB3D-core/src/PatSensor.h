//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: PatSensor
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 20th December 2021
Date Edited:  20th April 2022 
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
  PatSensor(){}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    // Ping the touch sensor to see if it is connected
    Wire.beginTransmission(ADDR_TOUCHSENS);
    if(Wire.endTransmission() != 0){
      Serial.println(F("PATSENSOR: Failed to initialise touch sensor."));
      _isEnabled = false;
    }
    else{
      Serial.println(F("PATSENSOR: Initialised touch sensor."));
      _isEnabled = true;
    }

    // Generate Random Numbers
    _sensPatCountThres = random(_sensPatCountThresMin,_sensPatCountThresMax+1);
    // Start Timers
    _disableButtonsTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_isEnabled){return;}

    // SENSOR: Check for button press unless disabled
    if(_buttonsEnabled){
      // 0x01 = b1, 0x02 = b2, 0x03 = b1+b2
      uint8_t valueButton = 0;
      _touchSens.get_touch_button_value(&valueButton);
      if(~_patFlag && _disableButtonsTimer.finished()){
        if(valueButton & 0x01){
          _patFlag = true;
          _buttonOneFlag = true;
        }
        else if(valueButton & 0x02){
          _patFlag = true;
          _buttonTwoFlag = true;
        }
        else{
          _buttonOneFlag = false;
          _buttonTwoFlag = false;
        }
      }
    }
    else{
      _buttonOneFlag = false;
      _buttonTwoFlag = false;
    }
  }

  //---------------------------------------------------------------------------
  // ACCEPT PATS - called during the main during decision tree
  void acceptPats(){
    if(!_isEnabled){return;}

    // Slider value, left=100, right=0
    uint8_t valueSlider = 0; 
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
  bool getEnabledFlag(){return _isEnabled;}
  bool getPatFlag(){return _patFlag;}
  bool getButtonOneFlag(){return _buttonOneFlag;}
  bool getButtonTwoFlag(){return _buttonTwoFlag;}
  uint8_t getPatCount(){return _sensPatCount;}

  bool getPatFinished(){
    return (_sensPatCount >= _sensPatCountThres);
  }

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}
  void setPatFlag(bool inFlag){_patFlag = inFlag;}
  void setPatCountThres(uint8_t inCount){_sensPatCountThres = inCount;}
  void setButtonsEnabled(bool inFlag){_buttonsEnabled = inFlag;}
  
  //---------------------------------------------------------------------------
  // RESET FUNCTION
  void reset(){
    _patFlag = false;   // Reset the pat flag
    _sensPatCount = 0;  // Reset the pat counter 
    genPatCountThres(); // Generate a new threshold
    // Disable buttons once pat is over
    _disableButtonsTimer.start(_disableButtonsTime); 
  }

  //---------------------------------------------------------------------------
  // GENERATE FUNCTIONS
  void genPatCountThres(){
    _sensPatCountThres = random(_sensPatCountThresMin,_sensPatCountThresMax+1);  
  }

private:
  //---------------------------------------------------------------------------
  // TOUCH SENSOR
  CY8C _touchSens = CY8C();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

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
