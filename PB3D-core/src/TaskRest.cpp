//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskRest
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "TaskRest.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskRest::TaskRest(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskRest::begin(){
    _timerObj.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskRest::update(){
    // If the task has changed then modify other classes as needed
    if(_taskObj->getNewTaskFlag()){
        reset();
        _speakerObj->reset();
    }
}

//---------------------------------------------------------------------------
// REST
void TaskRest::rest(){
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

//---------------------------------------------------------------------------
// Get, set and reset
void TaskRest::reset(){
    _timerObj.start(0);
    _restLEDVal = _restLEDMax;
    _restLEDIncrease = false;
    _speakerObj->reset();
}
  