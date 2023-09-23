//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TASKINTERACT
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 7th February 2021
Date Edited:  8th October 2021
*/

#ifndef TASKINTERACT_H
#define TASKINTERACT_H

#include <Wire.h> // I2C 
#include <Seeed_CY8C401XX.h> // Capcitive Touch Sensor
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "TaskDance.h"
#include "Timer.h"
#include "PatSensor.h"

class TaskInteract{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskInteract(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, 
               Speaker* inSpeaker, TaskDance* inDance, PatSensor* inPatSens){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _taskDanceObj = inDance;
    _speakerObj = inSpeaker;
    _patSensObj = inPatSens;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    _askSqueakTimer.start(0);
    _askWiggleTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update(){
    if(!_isEnabled){return;}
    
    // SENSOR: Check for start of pat
    if(_patSensObj->getButtonTwoFlag()){
      _taskObj->setTask(TASK_INTERACT);
      _taskObj->setTaskDuration(_patTimeOut);
    }
  }

  //---------------------------------------------------------------------------
  // INTERACT - called during the main during decision tree
  void interact(){
    // Set the LEDs on every loop
    _taskObj->taskLEDInteract();
    
    // If this is the first time we enter the function set key variables
    if(_interactStartFlag){
      _interactStartFlag = false;
      
      _moveObj-> stop();
      _moveObj->resetSubMoveTimer();
      
      _patSensObj->reset();
      _patSensObj->setButtonsEnabled(false);
      _patTimeOutTimer.start(_patTimeOut);
  
      _askFlag = true;
      _askSqueakTimer.start(_askSqueakInterval);
      _askWiggleTimer.start(_askWiggleDuration);

      _speakerObj->reset();
    }

    // Set the speaker codes on every loop
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speakerObj->setSoundCodes(inCodes,4);
    //uint16_t inFreqs[]  = {NOTE_C5,NOTE_C7,NOTE_C4,NOTE_G7,0,0,0,0};
    uint16_t inFreqs[]  = {NOTE_B4,NOTE_B6,NOTE_B3,NOTE_F7,0,0,0,0};
    uint16_t inDurs[]   = {200,200,300,200,0,0,0,0};
    _speakerObj->setSoundFreqs(inFreqs,8);
    _speakerObj->setSoundDurs(inDurs,8);

    // Check if we need to ask for a pat again
    if(_askSqueakTimer.finished()){
      _askFlag = true;
      _askSqueakTimer.start(_askSqueakInterval);
      _askWiggleTimer.start(_askWiggleDuration);
      
      // Reset the speaker 
      _speakerObj->reset();
    }
    
    if(_askFlag){
      // If we are done 'asking' reset the flag
      if(_askWiggleTimer.finished()){
        _askFlag = false;
      }
      else{
        // Otherwise 'wiggle' to ask for a pat
        _moveObj->wiggle(_askWiggleLeftDur,_askWiggleRightDur);     
      }
    }
    else{
      // Wait for a pat until next wiggle time
      _moveObj->stop();
    }
  
    //-----------------------------------------------------------------------
    // ACCEPT PATS
    _patSensObj->acceptPats();

    if(_patSensObj->getPatFinished()){
      // INTERACT EXIT CONDITION - reset start flag
      _interactStartFlag = true;
      
      // Reset the pat sensor
      _patSensObj->reset();
      _patSensObj->setButtonsEnabled(true);
      
      // Update mood to happy
      int8_t prob = random(0,100);
      if(prob <= 80){
        _moodObj->setMood(MOOD_HAPPY);
      }
      else{
        _moodObj->setMood(MOOD_NEUTRAL);
      }
      _moodObj->incMoodScore();      
      
      // Update task to dance
      _taskObj->setTask(TASK_DANCE);
      // Overide default task duration to be a specific number of bars
      _taskObj->setTaskDuration(round(4*_taskDanceObj->getDanceBarMs()));
      _taskObj->setDanceUpdateFlag(false);
      _taskDanceObj->setStartFlag(true);
      _taskDanceObj->setSpeakerFlag(true);
    }
  
    // Check for timeout, if so set mood to sad and explore
    if(_patTimeOutTimer.finished()){
      // INTERACT EXIT CONDITION - reset start flag
      _interactStartFlag = true;

      // Reset the pat sensor
      _patSensObj->reset();
      _patSensObj->setButtonsEnabled(true);
      
      // Update mood to sad
      int8_t prob = random(0,100);
      if(prob <= 75){ 
        _moodObj->setMood(MOOD_SAD);
      }
      else{
        _moodObj->setMood(MOOD_NEUTRAL);
      }
      _moodObj->decMoodScore();
      
      // Update task to explore
      _taskObj->setTask(TASK_EXPLORE);
    }
  }

  //-----------------------------------------------------------------------
  // GETTERS
  bool getEnabledFlag(){return _isEnabled;}
  uint16_t getTimeOut(){return _patTimeOut;}

  //-----------------------------------------------------------------------
  // SETTERS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}
  
  void setStartInteractFlag(bool inFlag){
    _interactStartFlag = inFlag;
    _patSensObj->setPatFlag(inFlag);
  }
  
private:
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj;
  TaskManager* _taskObj;
  MoveManager* _moveObj;
  TaskDance* _taskDanceObj;
  Speaker* _speakerObj;
  PatSensor* _patSensObj; 

  // TASK - INTERACT - Enabled Flag
  bool _isEnabled = true;

  // TASK - INTERACT Variables
  bool _interactStartFlag = true;
 
  // 'Ask' update variables
  bool _askFlag = false;
  uint16_t _askSqueakInterval = 7000;
  Timer _askSqueakTimer = Timer();
  uint16_t _askWiggleLeftDur = 500;
  uint16_t _askWiggleRightDur = 500;
  uint16_t _askWiggleDuration = 2*(_askWiggleLeftDur+_askWiggleRightDur);
  Timer _askWiggleTimer = Timer();
 
 
  uint16_t _patTimeOut = 30000;
  Timer _patTimeOutTimer = Timer();
};
#endif // TASKINTERACT
