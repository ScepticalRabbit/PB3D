//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskDance.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskDance::TaskDance(MoodManager* inMood, TaskManager* inTask,
                     MoveManager* inMove, Speaker* inSpeaker){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskDance::begin(){
    _danceStartFlag = true;
    _timerObj.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every loop
void TaskDance::update(){
    // Get new task flag reset after first main loop update
    if(_task_manager->get_new_task_flag()){
        // Reset the dance start flag in case this is the current task.
        _danceStartFlag = true;

        if(_task_manager->get_task() == TASK_DANCE){
        _generateTempo();
        _generateDance();
        _task_manager->set_dance_duration(_dance_duration);
        _task_manager->set_task_duration(_dance_duration);
        }
    }
}

//---------------------------------------------------------------------------
// DANCE
void TaskDance::dance(){
    // At the start of the dance set and reset key variables
    if(_danceStartFlag){
        _startDance();
    }

    // Set the task LEDs on every loop
    _task_manager->task_LED_dance();

    // If needed set the speaker flags on every loop
    if(_speakerFlag){
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
        _speaker->setSoundCodes(inCodes,4);
    }

    // Update the dance move at given time interval, should be 1/4 note
    if(_timerObj.finished()){
        _updateDance();
    }

    // MAIN DANCE DECISION TREE
    if(_danceCurrMove == DANCE_STOP){
        _move_manager->stop();
    }
    else if(_danceCurrMove == DANCE_FORBACK){
        //Serial.println("DANCE_FORBACK");
        _move_manager->forward_back(int(_dance4NoteMs),int(_dance4NoteMs));
    }
    else if(_danceCurrMove == DANCE_CIRCLE){
        //Serial.println("DANCE_CIRCLE");
        _move_manager->circle();
    }
    else if(_danceCurrMove == DANCE_TURN){
        //Serial.println("DANCE_TURN");
        if(_danceTurnDir == MOVE_B_RIGHT){
        //Serial.println("DANCE TURN RIGHT.");
        _move_manager->turn_to_angle_ctrl_speed(90.0);
        }
        else{
        //Serial.println("DANCE TURN LEFT.");
        _move_manager->turn_to_angle_ctrl_speed(-90.0);
        }
    }
    else if(_danceCurrMove == DANCE_SPIN){
        //Serial.println("DANCE_SPIN");
        if(_danceSpinDir == MOVE_B_RIGHT){
        _move_manager->right();
        }
        else{
        _move_manager->left();
        }
    }
    else{
        //Serial.println("DANCE_WIGGLE");
        _move_manager->wiggle(int(_dance4NoteMs),int(_dance4NoteMs));
    }
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
void TaskDance::_startDance(){
    _danceStartFlag = false;
    // Set initial variables
    _timerObj.start(round(_danceBarMs));
    _danceCurrBar = 0;
    _danceMoveInd = 0;
    _danceCurrMove = _danceMoveVec[_danceMoveInd];
    _move_manager->reset_submove_timer();

    // Increase mood when dance starts
    _mood_manager->inc_mood_score();

    // Set speaker
    if(_speakerFlag){
        _speaker->reset();
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
        _speaker->setSoundCodes(inCodes,4);
        uint16_t inFreqs[]  = {NOTE_G4,NOTE_G7,NOTE_G5,NOTE_G7,0,0,0,0};
        uint16_t inDurs[]   = {200,150,300,150,0,0,0,0};
        _speaker->setSoundFreqs(inFreqs,8);
        _speaker->setSoundDurs(inDurs,8);
    }
}

void TaskDance::_generateTempo(){
    _danceBPM = float(random(_danceBPMMin,_danceBPMMax)); // NOTE: random num between (min,max-1)
    _dance4NoteMs = (60.0/_danceBPM)*1000.0;
    _danceBarMs = _dance4NoteMs*4.0;
    _dance_duration = uint32_t(_danceBarMs*float(_danceNumBars));
}

void TaskDance::_generateDance(){
    // Random move that recurs throughout the dance
    uint8_t recurMove = random(1,3);  // NOTE: random num between (min,max-1)
    // Bars on which the recurrent move occurs, 0=B1 and B3, 1=B2 and B4
    uint8_t recurBeat = random(0,2); // NOTE: random num between (min,max-1)
    // 1 = Full 8 bar dance, 2 = Repeat 4 bar dance
    uint8_t recurBars = random(1,3); // NOTE: random num between (min,max-1)

    recurMove = 1;
    recurBeat = 0;
    //recurBars = 2;

    for(uint8_t ii=0 ; ii < _danceNumMoves ; ii++){
        if((recurBars == 2)&&(ii >= (_danceNumMoves/recurBars))){
        _danceMoveVec[ii] = _danceMoveVec[ii-(_danceNumMoves/recurBars)];
        }
        else{
        // If this is the recurring move set it, else generate
        if((ii%2)==recurBeat){
            _danceMoveVec[ii] = recurMove;
        }
        else{
            _danceMoveVec[ii] = random(1,DANCE_NUM_MOVES);
        }
        }
    }
    }

void TaskDance::_updateDance(){
    _timerObj.start(round(_danceBarMs));

    // Update the dance move
    _danceMoveInd = _danceMoveInd+1;
    if(_danceMoveInd >= _danceNumMoves){
        _danceMoveInd = 0;
    }
    _danceCurrMove = _danceMoveVec[_danceMoveInd];

    // Update the dance bar
    _danceCurrBar = _danceCurrBar+1;
    // If the dance is over reset the count
    if(_danceCurrBar >= _danceNumBars){
        _danceStartFlag = true;
        _danceCurrBar = 0;
    }

    // Update turn/spin direction
    if(_danceMoveInd == 0){
        _danceTurnDir = MOVE_B_LEFT;
        _danceSpinDir = MOVE_B_LEFT;
    }
    else if(_danceMoveVec[_danceMoveInd-1] == DANCE_TURN){
        if(_danceTurnDir == MOVE_B_RIGHT){
        _danceTurnDir = MOVE_B_LEFT;
        }
        else{
        _danceTurnDir = MOVE_B_RIGHT;
        }
    }
    else if(_danceMoveVec[_danceMoveInd-1] == DANCE_SPIN){
        if(_danceSpinDir = MOVE_B_RIGHT){
        _danceSpinDir = MOVE_B_LEFT;
        }
        else{
        _danceSpinDir = MOVE_B_RIGHT;
        }
    }

    // Reset the speaker to sync with dance bars
    _speaker->reset();
}

