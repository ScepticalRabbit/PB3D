#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskTantrum.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskTantrum.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskTantrum::TaskTantrum(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _mood_manager = inMood;
    _move_manager = inMove;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
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
    _timerObj1.start(_tantrum_duration);
    // Growl timer and flag
    _growlFlag = true;
    _timerObj2.start(_tantrumGrowlDuration);

    // MOOD UPDATE: 30% chance of angry
    int8_t prob = random(0,100);
    if(prob<30){_mood_manager->set_mood(MOOD_ANGRY);}
    _mood_manager->dec_mood_score();

    _speaker->reset();
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->setSoundCodes(inCodes,4);
    uint16_t inDurs[]   = {600,300,600,300,0,0,0,0};
    _speaker->setSoundDurs(inDurs,8);
}

// Set the speaker codes on every loop
uint8_t inCodes[]   = {SPEAKER_GROWL,SPEAKER_GROWL,SPEAKER_OFF,SPEAKER_OFF};
_speaker->setSoundCodes(inCodes,4);

// Set the task LEDs on every loop regardless
_task_manager->task_LED_tantrum();

if(_timerObj2.finished()){
    _growlFlag = !_growlFlag;
    if(_growlFlag){
    _timerObj2.start(_tantrumGrowlDuration);
    _speaker->reset();
    }
    else{
    _timerObj2.start(_tantrumGrowlPause);
    }
}
if(_timerObj1.finished()){
    _tantrumComplete = true;
}
else{
    _move_manager->forward_back(_tantrumFBDuration,_tantrumFBDuration);
}
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskTantrum::setStartTantrumFlag(){
    _startTantrumFlag = true;
    _tantrumComplete = false;
}