//---------------------------------------------------------------------------
// PET BOT - PB3! 
// CLASS: TASKREST
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef TASKREST_H
#define TASKREST_H

#include <Arduino.h>
#include "Task.h"
#include "MoveManager.h"
#include "Timer.h"

class TaskRest{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskRest(MoodManager* inMood, Task* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    _timerObj.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update(){
    // If the task has changed then modify other classes as needed
    if(_taskObj->getNewTaskFlag()){
      reset();
      _speakerObj->reset();
    }
  }

  void rest(){
    // Reset the speaker flags
    uint8_t inCodes[] = {SPEAKER_SNORE,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
    _speakerObj->setSoundCodes(inCodes,4);

    // Stop moving while sleeping
    _moveObj->stop();
    
    if(_timerObj.finished()){
      if(_restLEDIncrease){
        // If we are increasing increment the LED intensity
        _restLEDVal = _restLEDVal+1;
        
        // If we are above the max value reset and decrease
        if(_restLEDVal>=_restLEDMax){
          _restLEDVal = _restLEDMax;
          _restLEDIncrease = false;
          _speakerObj->reset();
        }
      }
      else{
        // If we are decreasing decrement the LED intensity
        _restLEDVal = _restLEDVal-1;
        
        // If we are below the min intensity reset and increase
        if(_restLEDVal<=_restLEDMin){
          _restLEDVal = _restLEDMin;
          _restLEDIncrease = true;
        }  
      }
      // Restart the timerr
      _timerObj.start(_restUpdateTime);
      // Set the current LED value
      _taskObj->taskLEDRest(_restLEDVal);
    }
  }

  void reset(){
    _timerObj.start(0);
    _restLEDVal = _restLEDMax;
    _restLEDIncrease = false;
    _speakerObj->reset();
  }
  
private:
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj;
  Task* _taskObj;
  MoveManager* _moveObj;
  Speaker* _speakerObj;

  // Timers
  Timer _timerObj = Timer();

  // SUBTASK - REST Variables
  // Use to pulse LEDs during rest task
  uint8_t _restUpdateTime = 7;
  bool _restLEDIncrease = true;
  uint8_t _restLEDMax = 254;
  uint8_t _restLEDMin = 0;
  uint8_t _restLEDVal = 254;
};
#endif // TASKREST
