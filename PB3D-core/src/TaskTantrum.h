//---------------------------------------------------------------------------
// PET BOT - PB3! 
// CLASS: TASKTANTRUM
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 29th Aug 2021
Date Edited:  29th Aug 2021 
*/

#ifndef TASKTANTRUM_H
#define TASKTANTRUM_H

#include <Arduino.h>
#include "MoodManager.h"
#include "Task.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

class TaskTantrum{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskTantrum(MoodManager* inMood, Task* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _moveObj = inMove;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
  }
  
  void setStartTantrumFlag(){
    _startTantrumFlag = true;
    _tantrumComplete = false;
  }

  void haveTantrum(){
    if(_startTantrumFlag){
      _startTantrumFlag = false;
      // Tantrum timer and completion
      _tantrumComplete = false;
      _timerObj1.start(_tantrumDuration);
      // Growl timer and flag
      _growlFlag = true;
      _timerObj2.start(_tantrumGrowlDuration);
      
      // MOOD UPDATE: 30% chance of angry
      int8_t prob = random(0,100);
      if(prob<30){_moodObj->setMood(MOOD_ANGRY);}
      _moodObj->decMoodScore();

      _speakerObj->reset();
      uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
      _speakerObj->setSoundCodes(inCodes,4);
      uint16_t inDurs[]   = {600,300,600,300,0,0,0,0};
      _speakerObj->setSoundDurs(inDurs,8);
    }

    // Set the speaker codes on every loop
    uint8_t inCodes[]   = {SPEAKER_GROWL,SPEAKER_GROWL,SPEAKER_OFF,SPEAKER_OFF};
    _speakerObj->setSoundCodes(inCodes,4);

    // Set the task LEDs on every loop regardless
    _taskObj->taskLEDTantrum();

    if(_timerObj2.finished()){
      _growlFlag = !_growlFlag;
      if(_growlFlag){
        _timerObj2.start(_tantrumGrowlDuration);
        _speakerObj->reset();  
      }
      else{
        _timerObj2.start(_tantrumGrowlPause); 
      }
    }
    if(_timerObj1.finished()){
      _tantrumComplete = true;
    }
    else{
      _moveObj->forwardBack(_tantrumFBDuration,_tantrumFBDuration);
    }
  }

  uint8_t getThreshold(){return _tantrumThreshold;}
  uint16_t getDuration(){return _tantrumDuration;}
  bool getCompleteFlag(){return _tantrumComplete;}

private:
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj;
  Task* _taskObj;
  MoveManager* _moveObj;
  Speaker* _speakerObj;
  // Timers
  Timer _timerObj1 = Timer();
  Timer _timerObj2 = Timer();
  
  // SUBTASK - TANTRUM Variables
  bool _startTantrumFlag = false;
  bool _tantrumComplete = false;
  uint16_t _tantrumThreshold = 31;
  uint16_t _tantrumFBDuration = 400;
  uint8_t _tantrumFBNum = 5;
  uint16_t _tantrumDuration = 2*_tantrumFBNum*_tantrumFBDuration;

  bool _growlFlag = true;
  uint16_t _tantrumGrowlDuration = 2*_tantrumFBDuration;
  uint16_t _tantrumGrowlPause = 1*_tantrumFBDuration;
};
#endif // TASKTANTRUM
