//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS - TASKFINDLIGHT
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 26th March 2022
Date Edited:  26th March 2022 
*/
#ifndef TASKFINDLIGHT_H
#define TASKFINDLIGHT_H

#include <Wire.h> // I2C
#include "Mood.h"
#include "Task.h"
#include "Move.h"
#include "Timer.h"
#include "Speaker.h"
#include "PatSensor.h"

// TASKFINDLIGHT: specific defines/includes
#define TCAADDR 0x70
#define LIGHTSENS_L 1
#define LIGHTSENS_R 0 
#include "Adafruit_VEML7700.h"

class TaskFindLight{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskFindLight(Mood* inMood, Task* inTask, Move* inMove, 
                Speaker* inSpeaker, PatSensor* inPatSens){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
    _patSensObj = inPatSens;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // LEFT light sensor
    _tcaSelect(LIGHTSENS_L);
    if(!_lightSensL.begin(&Wire)) {
      Serial.println(F("TASKFINDLIGHT: Left light sensor NOT found."));
      _isEnabled = false;
      //while(true){};
    }
    Serial.println(F("TASKFINDLIGHT: Left light sensor found."));
    _lightSensL.setGain(VEML7700_GAIN_1);
    _lightSensL.setIntegrationTime(VEML7700_IT_200MS);

    // RIGHT light sensor
    _tcaSelect(LIGHTSENS_R);
    if(!_lightSensR.begin(&Wire)) {
      Serial.println(F("TASKFINDLIGHT: Right light sensor NOT found."));
      _isEnabled = false;
      //while(true){};
    }
    Serial.println(F("TASKFINDLIGHT: Right light sensor found."));
    _lightSensR.setGain(VEML7700_GAIN_1);
    _lightSensR.setIntegrationTime(VEML7700_IT_200MS);

    // Start timers
    _senseTimer.start(_senseUpdateTime);
    _gradTimer.start(_gradUpdateTime);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_isEnabled){return;}

    if(_senseTimer.finished()){
      _senseTimer.start(_senseUpdateTime);
      
      _tcaSelect(LIGHTSENS_L);
      _luxLeft = _lightSensL.readLux(VEML_LUX_NORMAL_NOWAIT);  
      _tcaSelect(LIGHTSENS_R);
      _luxRight = _lightSensR.readLux(VEML_LUX_NORMAL_NOWAIT);

      _luxDiff = _luxLeft-_luxRight;
      _luxAvg = (_luxLeft+_luxRight)/2;
      _luxThresMid = _luxAvg*_luxPcThresMid;
      _luxThresStr = _luxAvg*_luxPcThresStr;
    }

    if(_gradTimer.finished()){
      _gradTimer.start(_gradUpdateTime);

      _luxLRAvgT0 = _luxLRAvgT1; 
      _luxLRAvgT1 = _luxAvg;

      _luxGrad = _luxLRAvgT1 - _luxLRAvgT0;
      _luxTAvg = (_luxLRAvgT1 + _luxLRAvgT0)/2;
      _luxGradThres = _luxGradPcThres*_luxTAvg;
    }

    if(_patSensObj->getButtonOneFlag()){
      _taskObj->setTask(TASK_FINDLIGHT);
      // Increase mood score when asked to play light game
      _moodObj->incMoodScore();
    }
  }

  //---------------------------------------------------------------------------
  // FINDLIGHT - called during the main during decision tree
  void findLight(){
    _taskObj->taskLEDFindLight();
    if(!_isEnabled){return;}

    _findLux(true);
  }

  void findDark(){
    _taskObj->taskLEDFindDark();
    if(!_isEnabled){return;}

    _findLux(false);
  }

  //---------------------------------------------------------------------------
  // UTILITY FUNCTIONS
  void resetGrad(){
    _gradTimer.start(_gradUpdateTime);
    _gradMoveFlag = false;
    _luxLRAvgT0 = _luxAvg; // Set both T0 and T1 to current LR avg
    _luxLRAvgT1 = _luxAvg;
    _luxGrad = _luxLRAvgT1 - _luxLRAvgT0;
    _luxTAvg = (_luxLRAvgT1 + _luxLRAvgT0)/2;
    _luxGradThres = _luxGradPcThres*_luxTAvg;
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}
  float getLuxLeft(){return _luxLeft;}
  float getLuxRight(){return _luxRight;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  
  void _findLux(bool seekLightFlag){
    // Check if diff thresholds tripped and set turn rad based on threshold
    bool thresTrip = false;
    float speedDiffFrac = 0.6;
    if(abs(_luxDiff) >= _luxThresMid){
      thresTrip = true;
      speedDiffFrac = 0.6;
    }
    else if(abs(_luxDiff) >= _luxThresStr){
      thresTrip = true;
      speedDiffFrac = 0.9;
    }

    // Check the temporal gradient
    if(abs(_luxGrad) >= _luxGradThres){
      if((_luxGrad > 0) && seekLightFlag){
        resetGrad(); // Resets params but sets move flag false
        _gradMoveTimeout.start(_gradMoveTimeoutTime);
        _gradMoveFlag = true; // Force the move flag back to false
      }
      else if((_luxGrad < 0) && !seekLightFlag){
        resetGrad(); // Resets params but sets move flag false
        _gradMoveTimeout.start(_gradMoveTimeoutTime);
        _gradMoveFlag = true; // Force the move flag back to false
      }
    }

    // If either threshold was tripped then turn
    if(_gradMoveFlag){
      _moveObj->turnToAngleCtrlPos(180.0);
      
      if(_moveObj->getPosPIDAttainSP_Both() || _gradMoveTimeout.finished()){
        _gradMoveFlag = false;
      }
    }
    else if(thresTrip){  
      if(_luxDiff > 0){
        if(seekLightFlag){ // Turn towards light
          _moveObj->forwardLeftDiffFrac(speedDiffFrac);
        }
        else{ // Turn away from light
          _moveObj->forwardRightDiffFrac(speedDiffFrac);
        }
      }
      else{
        if(seekLightFlag){ // Turn towards light
          _moveObj->forwardRightDiffFrac(speedDiffFrac);
        }
        else{ // Turn away from light
          _moveObj->forwardLeftDiffFrac(speedDiffFrac);
        }
      }
    }
    else{
      _moveObj->forward();  
    }
  }

  void _tcaSelect(uint8_t index) {
    if (index > 7) return;
    
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << index);
    Wire.endTransmission();  
  }

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
  Speaker* _speakerObj;
  PatSensor* _patSensObj;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

  Adafruit_VEML7700 _lightSensL = Adafruit_VEML7700();
  Adafruit_VEML7700 _lightSensR = Adafruit_VEML7700();

  uint16_t _senseUpdateTime = 400;
  Timer _senseTimer = Timer();
  float _luxLeft = 0.0, _luxRight = 0.0;
  float _luxAvg = 0.0, _luxDiff = 0.0;
  float _luxPcThresMid = 0.2, _luxThresMid = 0.0;
  float _luxPcThresStr = 0.5, _luxThresStr = 0.0;
  float _luxThresStop = 20.0;

  uint16_t _gradUpdateTime = 5000;
  Timer _gradTimer = Timer();
  float _luxLRAvgT0 = 0.0, _luxLRAvgT1 = 0.0;
  float _luxTAvg = 0.0;
  float _luxGrad = 0.0;
  float _luxGradPcThres = 0.2, _luxGradThres = 0.0;
  bool _gradMoveFlag = false;
  Timer _gradMoveTimeout = Timer();
  uint16_t _gradMoveTimeoutTime = 2000;
};
#endif // End TASKFINDLIGHT
