#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskManager.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskManager.h"

TaskManager::TaskManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){
    _task_LEDs = RGBLEDs;
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskManager::begin(){
    // Generate a probability, start the task timer and set the task
    _task_percent = random(0,100); // NOTE: random num between (min,max-1)
    _taskTimer.start(_task_duration);
    _LED_timer.start(0);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskManager::update(){
    if(_taskTimer.finished()){
        _update();
    }
}

void TaskManager::force_update(){
    _update();
}

//------------------------------------------------------------------------------
// Get, set and reset
// Default setTask function uses default task durations
void TaskManager::set_task(ETaskCode task){
    if(task != _task_code){
        _task_code = task;
        _task_new_flag = true;
        uint32_t tempDuration = random(_task_duration_min,_task_duration_max);

        //----------------------------------------------------------------------
        switch(_task_code){
            case TASK_TEST:
                _task_duration = 30000;
                task_LED_test();
                break;
            case TASK_EXPLORE:
                _task_duration = tempDuration;
                task_LED_explore();
                break;
            case TASK_REST:
                _task_duration = tempDuration;
                break;
            case TASK_DANCE:
                _task_duration = _dance_duration;
                task_LED_dance();
                break;
            case TASK_TANTRUM:
                _task_duration = _tantrum_duration;
                task_LED_collision();
                break;
            case TASK_FINDHUMAN:
                _task_duration = 20000;
                task_LED_find_human();
                break;
            case TASK_FINDLIGHT:
                _task_duration = 20000;
                task_LED_find_light();
                break;
            case TASK_FINDDARK:
                _task_duration = 20000;
                task_LED_find_dark();
                break;
            case TASK_FINDSOUND:
                _task_duration = 20000;
                task_LED_find_sound();
                break;
            case TASK_INTERACT:
                _task_duration = 20000;
                task_LED_interact();
                break;
            case TASK_PICKEDUP:
                _task_duration = 60000;
                // LEDs set based on state in the TaskPickedUp class
                break;
            case TASK_PAUSE:
                _task_duration = 60000;
                // LEDs set based on state in the TaskPause class
                break;
            case TASK_POUNCE:
                _task_duration = 20000;
                break;
            default:
                _task_code = TASK_EXPLORE;
                _task_duration = tempDuration;
                task_LED_explore();
                break;
        }

        // Start the task timer
        _taskTimer.start(_task_duration);
    }
}

void TaskManager::set_task_duration(uint32_t task_duration){
    _task_duration = task_duration;
    _taskTimer.start(_task_duration);
}

void TaskManager::assign_probability(EMoodCode mood){
    // Moods: [neutral,happy,sad,angry,scared]
    // test is used for forcing task probability
    switch(mood){
        case MOOD_NEUTRAL:
            _set_task_probability(_task_prob_neutral);
            break;
        case MOOD_HAPPY:
            _set_task_probability(_task_prob_happy);
            break;
        case MOOD_SAD:
            _set_task_probability(_task_prob_sad);
            break;
        case MOOD_ANGRY:
            _set_task_probability(_task_prob_angry);
            break;
        case MOOD_SCARED:
            _set_task_probability(_task_prob_scared);
            break;
        case MOOD_TEST:
            _set_task_probability(_task_prob_test);
            break;
        default:
            _set_task_probability(_task_prob_neutral);
            break;
    }
}

//------------------------------------------------------------------------------
// TASK LED FUNCTIONS
void TaskManager::task_LED_collision(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 0, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 0, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_explore(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 255, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 255, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_rest(uint8_t intensity){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 0, intensity));
    _task_LEDs->setPixelColor(1, _task_LEDs->Color(0, 0, intensity));
    _task_LEDs->setPixelColor(2, _task_LEDs->Color(0, 0, intensity));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 0, intensity));
    _task_LEDs->show();
}

void TaskManager::task_LED_dance(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 255, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 255, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_tantrum(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 0, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 0, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_find_human(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 0, 255));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 0, 255));
    _task_LEDs->show();
}

void TaskManager::task_LED_interact(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 255, 255));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 255, 255));
    _task_LEDs->show();
}

void TaskManager::task_LED_pickedup_Ok(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 255, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 255, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_pickedup_panic(){
    if(_LED_timer.finished()){
        _LED_timer.start(_LED_on_off_time);
        _LED_switch = !_LED_switch;
    }

    if(_LED_switch){
        _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 0, 0));
        _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 0, 0));
        _task_LEDs->show();
    }
    else{
        task_LED_off();
    }
}

void TaskManager::task_LED_pause(uint16_t pause_time){
    if(_LED_timer.finished()){
        _LED_timer.start(pause_time);
    }

    uint8_t LEDVal = _calc_rising_LED_val(pause_time);
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(LEDVal, LEDVal, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(LEDVal, LEDVal, 0));
    _task_LEDs->show();
}

void TaskManager::task_LED_find_light(){
    uint8_t intens = 255;
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(intens, intens, intens));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(intens, intens, intens));
    _task_LEDs->show();
}

void TaskManager::task_LED_find_dark(){
    uint8_t intens = 85;
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(intens, intens, intens));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(intens, intens, intens));
    _task_LEDs->show();
}

void TaskManager::task_LED_find_sound(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 0, 255));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 0, 255));
    _task_LEDs->show();
}

void TaskManager::task_LED_test(){
    if(_LED_timer.finished()){
        _LED_timer.start(_LED_on_off_time);
        _LED_switch = !_LED_switch;
    }

    if(_LED_switch){
        _task_LEDs->setPixelColor(0, _task_LEDs->Color(255, 0, 0));
        _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 0, 255));
        _task_LEDs->show();
    }
    else{
        _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 0, 255));
        _task_LEDs->setPixelColor(3, _task_LEDs->Color(255, 0, 0));
        _task_LEDs->show();
    }
}

void TaskManager::task_LED_off(){
    _task_LEDs->setPixelColor(0, _task_LEDs->Color(0, 0, 0));
    _task_LEDs->setPixelColor(3, _task_LEDs->Color(0, 0, 0));
    _task_LEDs->show();
}

//------------------------------------------------------------------------
// HSV LEDS
void TaskManager::task_LED_hue(uint16_t hue){
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue)));
    _task_LEDs->show();
}

void TaskManager::task_LED_HSV(uint16_t hue, uint8_t sat, uint8_t value){
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue,sat,value)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue,sat,value)));
    _task_LEDs->show();
}

void TaskManager::task_LED_colour(uint16_t col){
    uint16_t hue = 65536*col/12;
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue)));
    _task_LEDs->show();
}

void TaskManager::task_LED_colour(uint16_t colL, uint16_t colR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hueR)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hueL)));
    _task_LEDs->show();
}

void TaskManager::task_LED_CSV(uint16_t col,
                               uint8_t sat,
                               uint8_t val){
    uint16_t hue = 65536*col/12;
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue,sat,val)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hue,sat,val)));
    _task_LEDs->show();
}

void TaskManager::task_LED_CSV(uint16_t colL,
                               uint16_t colR,
                               uint8_t satL,
                               uint8_t satR,
                               uint8_t valL,
                               uint8_t valR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _task_LEDs->setPixelColor(0, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hueR,satR,valR)));
    _task_LEDs->setPixelColor(3, _task_LEDs->gamma32(_task_LEDs->ColorHSV(hueL,satL,valL)));
    _task_LEDs->show();
}


void TaskManager::_update(){
    _task_percent = random(0,100); // NOTE: random num between (min,max-1)
    _dance_update_flag = false;

    if((_task_percent >= 0) &&
        (_task_percent < _task_prob_bounds[0])){ // EXPLORE
        set_task(TASK_EXPLORE);
    }
    else if((_task_percent >= _task_prob_bounds[0]) &&
            (_task_percent < _task_prob_bounds[1])){ // REST
        set_task(TASK_REST);
    }
    else if((_task_percent >= _task_prob_bounds[1]) &&
            (_task_percent < _task_prob_bounds[2])){ // DANCE
        _dance_update_flag = true;
        set_task(TASK_DANCE);
    }
    else if((_task_percent >= _task_prob_bounds[2]) &&
            (_task_percent < _task_prob_bounds[3])){ // FINDHUMAN
        set_task(TASK_FINDHUMAN);
    }
    else if((_task_percent >= _task_prob_bounds[3]) &&
            (_task_percent < _task_prob_bounds[4])){ // FINDSOUND
        set_task(TASK_FINDSOUND);
    }
    else if((_task_percent >= _task_prob_bounds[4]) &&
            (_task_percent < _task_prob_bounds[5])){ // FINDLIGHT
        set_task(TASK_FINDLIGHT);
    }
    else if((_task_percent >= _task_prob_bounds[5]) &&
            (_task_percent < _task_prob_bounds[6])){ // FINDDARK
        set_task(TASK_FINDDARK);
    }
    else{ // EXPLORE
        set_task(TASK_EXPLORE);
    }

    _taskTimer.start(_task_duration);
}

void TaskManager::_set_task_probability(const int8_t inProbs[]){
    int16_t probSum = 0;
    for(int8_t ii = 0; ii < _task_count; ii++){
        probSum = probSum+inProbs[ii];
        _task_prob_bounds[ii] = probSum;
    }
}

uint8_t TaskManager::_calc_falling_LED_val(uint16_t timeInt){
    float startVal = 255.0, endVal = 0.0;
    float slope = (float(startVal)-float(endVal))/(float(0.0)-float(timeInt));
    return round(float(startVal) + slope*float(_LED_timer.get_time()));
}

uint8_t TaskManager::_calc_rising_LED_val(uint16_t timeInt){
    float startVal = 0.0,  endVal = 255.0;
    float slope = (float(startVal)-float(endVal))/(float(0.0)-float(timeInt));
    return round(float(startVal) + slope*float(_LED_timer.get_time()));
}