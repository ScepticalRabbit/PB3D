//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: FIND SOUND
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 11th December 2021
Date Edited:  26th April 2022 
*/

#ifndef TASKFINDSOUND_H
#define TASKFINDSOUND_H

#include <Wire.h> // I2C 
#include "Mood.h"
#include "Task.h"
#include "Move.h"
#include "Timer.h"

// Define follower Xiao I2C Address
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif

// Communication flags from sub-board with 'ears'
#define EAR_COM_NOSOUND 0
#define EAR_COM_FORWARD 1
#define EAR_COM_LEFT 2
#define EAR_COM_RIGHT 3
#define EAR_COM_SENV 4

class TaskFindSound{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskFindSound(Mood* inMood, Task* inTask, Move* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    // Send the byte flag back
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(_sendByte);
    Wire.endTransmission();

    // Start all timers
    _sensUpdateTimer.start(0);
    _clapEnableTimer.start(0);
    _envSampTimer.start(0);
    _callTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during each loop to update variables
  void update(){
    // If the sensor wasn't found then do nothing
    if(!_isEnabled){return;}

    // SENSOR: Ask follower Xiao for sound data
    if(_sensUpdateTimer.finished()){
      _sensUpdateTimer.start(_sensUpdateTime);
      
      // Request ear state data from follower board
      _I2CReadEarState();
      
      // Send the byte flag back
      _I2CSendByte();

      if((_earState==EAR_COM_FORWARD)||(_earState==EAR_COM_LEFT)||(_earState==EAR_COM_RIGHT)){
        _clapCount++;
      }

      // DEBUG: Print to Serial
      /*
      Serial.print("E-STATE: ");
      Serial.print(_earState), Serial.print(", M: ");
      if(_earState==EAR_COM_FORWARD){Serial.print("F");}
      else if(_earState==EAR_COM_LEFT){Serial.print("L");}
      else if(_earState==EAR_COM_RIGHT){Serial.print("R");}
      else if(_earState==EAR_COM_SENV){Serial.print("E");}
      else{Serial.print("N");}
      Serial.println();
      */
    }

    if(_taskObj->getTask() != TASK_FINDSOUND){
      // ENABLE: If X claps in this interval then start finding sound 
      if(_clapEnableTimer.finished()){
        _clapEnableTimer.start(_clapEnableUpdateTime);
        
        if(_clapCount >= _clapThres){
          _taskObj->setTask(TASK_FINDSOUND);
        }
        _clapCount = 0;
      }

      // SAMPLE ENV: Resample environment to prevent false trips of the sensor
      if(_envSampTimer.finished()){
        _envSampTimer.start(_envSampUpdateTime);
        _I2CSendSampEnvFlag();
      }
    }
    else{
      _clapCount = 0;
    }

    // NEW TASK: if task is new set the start flag
    if(_taskObj->getNewTaskFlag()){
      _startFlag = true;
    }
  }

  //---------------------------------------------------------------------------
  // FINDSOUND - called during task decision tree
  void findSound(){
    // Set the LEDs on every loop regardless
    _taskObj->taskLEDFindSound();
    
    // If the sensor wasn't found then sxit the function
    if(!_isEnabled){return;}

    //--------------------------------------------------------------------
    // START
    // First time this is called as a new task reset some variables
    if(_startFlag){
      _startFlag = false;

      // Send the flag to sample environment
      _I2CSendSampEnvFlag();

      _speakerObj->reset();           
      _callTimer.start(_callInterval);
    }

    //--------------------------------------------------------------------
    // SPEAKER: call = where are you?
    // Set the speaker codes on every loop
    // uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    uint8_t inCodes[]   = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
    _speakerObj->setSoundCodes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_A4,NOTE_G4,NOTE_G4,NOTE_A6,0,0,0,0};
    uint16_t inDurs[]   = {300,0,200,0,0,0,0,0};
    _speakerObj->setSoundFreqs(inFreqs,8);
    _speakerObj->setSoundDurs(inDurs,8);    

    
    if(_callTimer.finished()){
      _speakerObj->reset();
      _callTimer.start(_callInterval);
    }
    
    //--------------------------------------------------------------------
    // FIND SOUND: Track based on ear state from Xiao
    // EAR STATE = 0: uncertain, 1: forward, 2: left, 3: right 
    if(_earState == EAR_COM_LEFT){
      //_moveObj->left();
      //_moveObj->forwardLeft(_speedDiffLR); 
      _moveObj->forwardLeftDiffFrac(_speedDiffFracLR);
    }
    else if(_earState == EAR_COM_RIGHT){ 
      //_moveObj->right();
      //_moveObj->forwardRight(_speedDiffLR);  
      _moveObj->forwardRightDiffFrac(_speedDiffFracLR);
    }
    else if(_earState == EAR_COM_FORWARD){ 
      _moveObj->forward();  
    }
    else if(_earState == EAR_COM_SENV){     
      _moveObj->forward(); 
    }
    else{
      //_moveObj->updateMove();
      //_moveObj->go();
      _moveObj->forward();  
    }
  }

  //---------------------------------------------------------------------------
  // GET/SET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}
  byte getEarState(){return _earState;}
  void setEarState(byte inState){_earState = inState;}
  byte getSendByte(){return _sendByte;}
  void setSendByte(byte inByte){_sendByte = inByte;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _I2CSendByte(){
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(_sendByte);
    Wire.endTransmission();
  }
 
  void _I2CSendSampEnvFlag(){
    byte sampEnvByte = _sendByte | B01000000;    
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(sampEnvByte);
    Wire.endTransmission();
  }

  void _I2CReadEarState(){
    Wire.requestFrom(ADDR_FOLLBOARD, 1);
    while (Wire.available()){
      _earState = Wire.read();
    }
  }

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  // Main Object Pointers
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
  Speaker* _speakerObj;

  // Base Varibles
  bool _isEnabled = true;
  bool _startFlag = false;

  // TASK - FIND SOUND VARS
  byte _earState = 0;
  byte _sendByte = B00000000;
  float _speedDiffLR = 75.0;
  float _speedDiffFracLR = 0.75;

  // Timer for controlling how often the sensor is sampled
  Timer _sensUpdateTimer = Timer();
  uint16_t _sensUpdateTime = 500;

  Timer _clapEnableTimer = Timer();
  uint16_t _clapEnableUpdateTime = 6000;
  uint8_t _clapThres = 3, _clapCount = 0;

  Timer _envSampTimer = Timer();
  uint16_t _envSampUpdateTime = 20000;

  // Variables for controlling calling: where are you?
  Timer _callTimer = Timer();
  uint16_t _callInterval = 4000;
};
#endif // FINDSOUND
