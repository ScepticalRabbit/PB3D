//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindLight.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskFindLight::TaskFindLight(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker, PatSensor* inPatSens){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
    _patSensObj = inPatSens;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindLight::begin(){
    // LEFT light sensor
    _tcaSelect(LIGHTSENS_L);
    if(!_lightSensL.begin(&Wire)) {
        Serial.println(F("TASKFINDLIGHT: Left light sensor NOT found."));
        _enabled = false;
        //while(true){};
    }
    Serial.println(F("TASKFINDLIGHT: Left light sensor found."));
    _lightSensL.setGain(VEML7700_GAIN_1);
    _lightSensL.setIntegrationTime(VEML7700_IT_200MS);

    // RIGHT light sensor
    _tcaSelect(LIGHTSENS_R);
    if(!_lightSensR.begin(&Wire)) {
        Serial.println(F("TASKFINDLIGHT: Right light sensor NOT found."));
        _enabled = false;
        //while(true){};
    }
    Serial.println(F("TASKFINDLIGHT: Right light sensor found."));
    _lightSensR.setGain(VEML7700_GAIN_1);
    _lightSensR.setIntegrationTime(VEML7700_IT_200MS);

    // Start timers
    _senseTimer.start(_senseUpdateTime);
    _gradTimer.start(_gradUpdateTime);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskFindLight::update(){
    if(!_enabled){return;}

    if(_senseTimer.finished()){
        _senseTimer.start(_senseUpdateTime);

        _tcaSelect(LIGHTSENS_L);
        _luxLeft = _lightSensL.readLux(VEML_LUX_NORMAL_NOWAIT);
        _tcaSelect(LIGHTSENS_R);
        _luxRight = _lightSensR.readLux(VEML_LUX_NORMAL_NOWAIT);

        _luxDiff = _luxLeft-_luxRight;
        _luxAvg = (_luxLeft+_luxRight)/2;
        _luxThresMid = _luxAvg*_luxPcThresMid;
        _luxThresStr = _luxAvg*_luxPcThresStr;
    }

    if(_gradTimer.finished()){
        _gradTimer.start(_gradUpdateTime);

        _luxLRAvgT0 = _luxLRAvgT1;
        _luxLRAvgT1 = _luxAvg;

        _luxGrad = _luxLRAvgT1 - _luxLRAvgT0;
        _luxTAvg = (_luxLRAvgT1 + _luxLRAvgT0)/2;
        _luxGradThres = _luxGradPcThres*_luxTAvg;
    }

    if(_patSensObj->get_button_one_flag()){
        _task_manager->set_task(TASK_FINDLIGHT);
        // Increase mood score when asked to play light game
        _mood_manager->inc_mood_score();
    }
}

//---------------------------------------------------------------------------
// FINDLIGHT - called during the main during decision tree
void TaskFindLight::findLight(){
    _task_manager->task_LED_find_light();
    if(!_enabled){return;}

    _findLux(true);
}

void TaskFindLight::findDark(){
    _task_manager->task_LED_find_dark();
    if(!_enabled){return;}

    _findLux(false);
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskFindLight::resetGrad(){
    _gradTimer.start(_gradUpdateTime);
    _gradMoveFlag = false;
    _luxLRAvgT0 = _luxAvg; // Set both T0 and T1 to current LR avg
    _luxLRAvgT1 = _luxAvg;
    _luxGrad = _luxLRAvgT1 - _luxLRAvgT0;
    _luxTAvg = (_luxLRAvgT1 + _luxLRAvgT0)/2;
    _luxGradThres = _luxGradPcThres*_luxTAvg;
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS

void TaskFindLight::_findLux(bool seekLightFlag){
    // Check if diff thresholds tripped and set turn rad based on threshold
    bool thresTrip = false;
    float speedDiffFrac = 0.6;
    if(abs(_luxDiff) >= _luxThresMid){
        thresTrip = true;
        speedDiffFrac = 0.6;
    }
    else if(abs(_luxDiff) >= _luxThresStr){
        thresTrip = true;
        speedDiffFrac = 0.9;
    }

    // Check the temporal gradient
    if(abs(_luxGrad) >= _luxGradThres){
        if((_luxGrad > 0) && seekLightFlag){
            resetGrad(); // Resets params but sets move flag false
            _gradMoveTimeout.start(_gradMoveTimeoutTime);
            _gradMoveFlag = true; // Force the move flag back to false
        }
        else if((_luxGrad < 0) && !seekLightFlag){
            resetGrad(); // Resets params but sets move flag false
            _gradMoveTimeout.start(_gradMoveTimeoutTime);
            _gradMoveFlag = true; // Force the move flag back to false
        }
    }

    // If either threshold was tripped then turn
    if(_gradMoveFlag){
        _move_manager->turn_to_angle_ctrl_pos(180.0);

        if(_move_manager->get_pos_PID_attained_set_point() || _gradMoveTimeout.finished()){
            _gradMoveFlag = false;
        }
    }
    else if(thresTrip){
        if(_luxDiff > 0){
            if(seekLightFlag){ // Turn towards light
                _move_manager->forward_left_diff_frac(speedDiffFrac);
            }
            else{ // Turn away from light
                _move_manager->forward_right_diff_frac(speedDiffFrac);
            }
        }
        else{
            if(seekLightFlag){ // Turn towards light
                _move_manager->forward_right_diff_frac(speedDiffFrac);
            }
            else{ // Turn away from light
                _move_manager->forward_left_diff_frac(speedDiffFrac);
            }
        }
    }
    else{
        _move_manager->forward();
    }
}

void TaskFindLight::_tcaSelect(uint8_t index) {
    if (index > 7) return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << index);
    Wire.endTransmission();
}