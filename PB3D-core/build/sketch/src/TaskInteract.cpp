#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskInteract.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskInteract.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskInteract::TaskInteract(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker, TaskDance* inDance, PatSensor* inPatSens){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _taskDanceObj = inDance;
    _speakerObj = inSpeaker;
    _patSensObj = inPatSens;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskInteract::begin(){
    _askSqueakTimer.start(0);
    _askWiggleTimer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskInteract::update(){
    if(!_is_enabled){return;}

    // SENSOR: Check for start of pat
    if(_patSensObj->getButtonTwoFlag()){
        _taskObj->setTask(TASK_INTERACT);
        _taskObj->setTaskDuration(_patTimeOut);
    }
}

//---------------------------------------------------------------------------
// INTERACT - called during the main during decision tree
void TaskInteract::interact(){
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

//---------------------------------------------------------------------------
// Get, set and reset
void TaskInteract::setStartInteractFlag(bool inFlag){
    _interactStartFlag = inFlag;
    _patSensObj->setPatFlag(inFlag);
}
