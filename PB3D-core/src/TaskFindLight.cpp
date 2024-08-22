//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindLight.h"


TaskFindLight::TaskFindLight(MoodManager* mood, TaskManager* task,
                             MoveManager* inMove, Speaker* speaker,
                             PatSensor* pat_sens){
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = inMove;
    _speaker = speaker;
    _pat_sensor = pat_sens;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindLight::begin(){
    // LEFT light sensor
    _multiplex_select(LIGHTSENS_L);
    if(!_light_sens_left.begin(&Wire)) {
        Serial.println(F("TASKFINDLIGHT: Left light sensor NOT found."));
        _enabled = false;
    }
    Serial.println(F("TASKFINDLIGHT: Left light sensor found."));
    _light_sens_left.setGain(VEML7700_GAIN_1);
    _light_sens_left.setIntegrationTime(VEML7700_IT_200MS);

    // RIGHT light sensor
    _multiplex_select(LIGHTSENS_R);
    if(!_light_sens_right.begin(&Wire)) {
        Serial.println(F("TASKFINDLIGHT: Right light sensor NOT found."));
        _enabled = false;
    }
    Serial.println(F("TASKFINDLIGHT: Right light sensor found."));
    _light_sens_right.setGain(VEML7700_GAIN_1);
    _light_sens_right.setIntegrationTime(VEML7700_IT_200MS);

    _sense_timer.start(_sens_update_time);
    _gradient_timer.start(_gradient_update_time);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskFindLight::update(){
    if(!_enabled){return;}

    if(_sense_timer.finished()){
        _sense_timer.start(_sens_update_time);

        _multiplex_select(LIGHTSENS_L);
        _lux_left = _light_sens_left.readLux(VEML_LUX_NORMAL_NOWAIT);
        _multiplex_select(LIGHTSENS_R);
        _lux_right = _light_sens_right.readLux(VEML_LUX_NORMAL_NOWAIT);

        _lux_diff = _lux_left-_lux_right;
        _lux_avg = (_lux_left+_lux_right)/2;
        _lux_threshold_mid = _lux_avg*_lux_percent_threshold_mid;
        _lux_thres_str = _lux_avg*_lux_percent_thres_str;
    }

    if(_gradient_timer.finished()){
        _gradient_timer.start(_gradient_update_time);

        _lux_avg_time0 = _lux_avg_time1;
        _lux_avg_time1 = _lux_avg;

        _lux_grad = _lux_avg_time1 - _lux_avg_time0;
        _lux_time_avg = (_lux_avg_time1 + _lux_avg_time0)/2;
        _lux_grad_thres = _lux_grad_percent_thres*_lux_time_avg;
    }

    if(_pat_sensor->get_button_one_flag()){
        _task_manager->set_task(TASK_FINDLIGHT);
        _mood_manager->inc_mood_score();
    }
}

//---------------------------------------------------------------------------
// FINDLIGHT - called during the main during decision tree
void TaskFindLight::find_light(){
    _task_manager->task_LED_find_light();
    if(!_enabled){return;}

    _find_lux(SEEK_LIGHT);
}

void TaskFindLight::find_dark(){
    _task_manager->task_LED_find_dark();
    if(!_enabled){return;}

    _find_lux(SEEK_DARK);
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskFindLight::reset_gradient(){
    _gradient_timer.start(_gradient_update_time);
    _grad_move_flag = false;
    _lux_avg_time0 = _lux_avg; // Set both T0 and T1 to current LR avg
    _lux_avg_time1 = _lux_avg;
    _lux_grad = _lux_avg_time1 - _lux_avg_time0;
    _lux_time_avg = (_lux_avg_time1 + _lux_avg_time0)/2;
    _lux_grad_thres = _lux_grad_percent_thres*_lux_time_avg;
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS

void TaskFindLight::_find_lux(EFindLight seekLightFlag){
    // Check if diff thresholds tripped and set turn rad based on threshold
    bool thresTrip = false;
    float speedDiffFrac = 0.6;
    if(abs(_lux_diff) >= _lux_threshold_mid){
        thresTrip = true;
        speedDiffFrac = 0.6;
    }
    else if(abs(_lux_diff) >= _lux_thres_str){
        thresTrip = true;
        speedDiffFrac = 0.9;
    }

    // Check the temporal gradient
    if(abs(_lux_grad) >= _lux_grad_thres){
        if((_lux_grad > 0) && seekLightFlag){
            reset_gradient(); // Resets params but sets move flag false
            _grad_move_timeout.start(_grad_move_timeout_time);
            _grad_move_flag = true; // Force the move flag back to false
        }
        else if((_lux_grad < 0) && !seekLightFlag){
            reset_gradient(); // Resets params but sets move flag false
            _grad_move_timeout.start(_grad_move_timeout_time);
            _grad_move_flag = true; // Force the move flag back to false
        }
    }

    // If either threshold was tripped then turn
    if(_grad_move_flag){
        _move_manager->turn_to_angle_ctrl_pos(180.0);

        if(_move_manager->get_pos_PID_attained_set_point() || _grad_move_timeout.finished()){
            _grad_move_flag = false;
        }
    }
    else if(thresTrip){
        if(_lux_diff > 0){
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

void TaskFindLight::_multiplex_select(uint8_t index) {
    if (index > 7) return;

    Wire.beginTransmission(ADDR_TCA_I2CMULTIPLEX);
    Wire.write(1 << index);
    Wire.endTransmission();
}