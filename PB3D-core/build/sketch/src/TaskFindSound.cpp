#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskFindSound.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindSound.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskFindSound::TaskFindSound(MoodManager* inMood, TaskManager* inTask,
                            MoveManager* inMove, Speaker* inSpeaker){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindSound::begin(){
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
void TaskFindSound::update(){
    // If the sensor wasn't found then do nothing
    if(!_is_enabled){return;}

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
        #ifdef DEBUG_TASKFINDSOUND
        Serial.print("E-STATE: ");
        Serial.print(_earState), Serial.print(", M: ");
        if(_earState==EAR_COM_FORWARD){Serial.print("F");}
        else if(_earState==EAR_COM_LEFT){Serial.print("L");}
        else if(_earState==EAR_COM_RIGHT){Serial.print("R");}
        else if(_earState==EAR_COM_SENV){Serial.print("E");}
        else{Serial.print("N");}
        Serial.println();
        #endif
    }

    if(_task_manager->getTask() != TASK_FINDSOUND){
        // ENABLE: If X claps in this interval then start finding sound
        if(_clapEnableTimer.finished()){
        _clapEnableTimer.start(_clapEnableUpdateTime);

        if(_clapCount >= _clapThres){
            _task_manager->setTask(TASK_FINDSOUND);
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
    if(_task_manager->getNewTaskFlag()){
        _start_flag = true;
    }
}

//---------------------------------------------------------------------------
// FINDSOUND - called during task decision tree
void TaskFindSound::findSound(){
    // Set the LEDs on every loop regardless
    _task_manager->taskLEDFindSound();

    // If the sensor wasn't found then sxit the function
    if(!_is_enabled){return;}

    //--------------------------------------------------------------------
    // START
    // First time this is called as a new task reset some variables
    if(_start_flag){
        _start_flag = false;

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
        //_move_manager->left();
        //_move_manager->forward_left(_speedDiffLR);
        _move_manager->forward_left_diff_frac(_speedDiffFracLR);
    }
    else if(_earState == EAR_COM_RIGHT){
        //_move_manager->right();
        //_move_manager->forward_right(_speedDiffLR);
        _move_manager->forward_right_diff_frac(_speedDiffFracLR);
    }
    else if(_earState == EAR_COM_FORWARD){
        _move_manager->forward();
    }
    else if(_earState == EAR_COM_SENV){
        _move_manager->forward();
    }
    else{
        //_move_manager->update_move();
        //_move_manager->go();
        _move_manager->forward();
    }
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
void TaskFindSound::_I2CSendByte(){
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(_sendByte);
    Wire.endTransmission();
}

void TaskFindSound::_I2CSendSampEnvFlag(){
    byte sampEnvByte = _sendByte | B01000000;
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(sampEnvByte);
    Wire.endTransmission();
}

void TaskFindSound::_I2CReadEarState(){
    Wire.requestFrom(ADDR_FOLLBOARD, 1);
    while (Wire.available()){
        _earState = Wire.read();
    }
}
