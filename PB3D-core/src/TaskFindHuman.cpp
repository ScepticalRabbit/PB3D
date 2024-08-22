//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindHuman.h"


TaskFindHuman::TaskFindHuman(MoodManager* inMood, TaskManager* inTask,
                             MoveManager* inMove, Speaker* inSpeaker,
                             TaskInteract* inTInt){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
    _task_interact = inTInt;
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindHuman::begin(){
    delay(500);
    _pres_detector = PresenceDetector(_movement_sens, _sensitivity_presence, _sensitivity_movement, _detect_interval);
    if (_movement_sens.initialize() == false){
        Serial.println(F("TASKFINDHUMAN: Failed to initialise presence sensor."));
        _enabled = false;
    }
    Serial.println(F("TASKFINDHUMAN: IR presence sensor initialised."));
    _IR_detect1 = false; _IR_detect2 = false; _IR_detect3 = false; _IR_detect4 = false;
    _sens_update_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskFindHuman::update(){
    if(!_enabled){return;}

    if(_task_manager->get_new_task_flag()){
        _start_flag = true;
    }

    // SENSOR: IR Human Presence Sensor
    _pres_detector.loop();
    if(_sens_update_timer.finished()){
        _update_human_IR_sensor();
        _sens_update_timer.start(_IR_sens_update_time);
    }
}

//---------------------------------------------------------------------------
// FIND HUMAN - called during task decision tree
void TaskFindHuman::find_human(){
    // Set the LEDs on every loop regardless
    _task_manager->task_LED_find_human();

    // If the human presence sensor wasn't found then bypass the rest of the function
    if(!_enabled){
        _task_manager->force_update();
        return;
    }

    // First time this is called as a new task reset some variables
    if(_start_flag){
        _start_flag = false;

        _speaker->reset();
        _call_timer.start(_call_interval);
    }


    // Set the speaker codes on every loop
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_A4,NOTE_A4,NOTE_A4,NOTE_CS7,0,0,0,0};
    uint16_t inDurs[]   = {300,0,200,0,0,0,0,0};
    _speaker->set_sound_freqs(inFreqs,8);
    _speaker->set_sound_durations(inDurs,8);

    //--------------------------------------------------------------------
    // Found you!
    //--------------------------------------------------------------------
    // NOTE SENSOR ORIENTATION DIFFERENT ON PB3D
    // Sensor 1 is pointing LEFT
    // Sensor 1: LEFT, Sensor 2: FRONT, Sensor 3: RIGHT, Sensor 4: BACK
    if(_IR_detect1 && _IR_detect2 && _IR_detect3 && _IR_detect4){
        // If all flags are tripped a human has been found!
        _task_manager->set_task(TASK_INTERACT);
        // Overide the default task duration
        _task_manager->set_task_duration(_task_interact->get_timeout());
        _task_manager->task_LED_interact();
        _task_interact->set_start_interact_flag(true);
        // Update mood score
        _mood_manager->inc_mood_score();
    }
    else{
        //--------------------------------------------------------------------
        // Speaker call = where you?
        //--------------------------------------------------------------------
        if(_call_timer.finished()){
            _speaker->reset();
            _call_timer.start(_call_interval);
        }

        //--------------------------------------------------------------------
        // Track based on IR sensor readings
        //--------------------------------------------------------------------
        // If grove connector points backwards then the sensors are:
        // 1: back, 2: left, 3: front, 4: right
        if(_diff_IR_1to3 > _sensitivity_track){
            // Sensor 1 greater than sensor 3
            _move_manager->left();
        }
        else if(_diff_IR_1to3 < -_sensitivity_track){
            // Sensor 3 greater than 1
            _move_manager->forward();
        }
        else if(_diff_IR_2to4 > _sensitivity_track){
            // Sensor 2 greater than sensor 4
            _move_manager->left();
        }
        else if(_diff_IR_2to4 < -_sensitivity_track){
            // Sensor 4 greater than sensor 2
            _move_manager->right();
        }
        else{
            _move_manager->update_move();
            _move_manager->go();
        }
    }
}

//---------------------------------------------------------------------------
void TaskFindHuman::_update_human_IR_sensor(){
    _pres_detector.presentFullField(false);

    _IR_deriv1 = _pres_detector.getDerivativeOfIR1();
    _IR_deriv2 = _pres_detector.getDerivativeOfIR2();
    _IR_deriv3 = _pres_detector.getDerivativeOfIR3();
    _IR_deriv4 = _pres_detector.getDerivativeOfIR4();

    _diff_IR_1to3 = _pres_detector.getDerivativeOfDiff13();
    _diff_IR_2to4 = _pres_detector.getDerivativeOfDiff24();

    _IR_detect1 = _pres_detector.presentField1();
    _IR_detect2 = _pres_detector.presentField2();
    _IR_detect3 = _pres_detector.presentField3();
    _IR_detect4 = _pres_detector.presentField4();
}