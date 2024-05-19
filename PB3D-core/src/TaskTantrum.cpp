//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskTantrum
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "TaskTantrum.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskTantrum::TaskTantrum(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _moveObj = inMove;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskTantrum::begin(){
    _timerObj1.start(0);
    _timerObj2.start(0);
}

//---------------------------------------------------------------------------
// TANTRUM
void TaskTantrum::haveTantrum(){
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

//---------------------------------------------------------------------------
// Get, set and reset
void TaskTantrum::setStartTantrumFlag(){
    _startTantrumFlag = true;
    _tantrumComplete = false;
}