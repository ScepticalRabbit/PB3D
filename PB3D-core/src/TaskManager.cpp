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
    _taskLEDs = RGBLEDs;
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskManager::begin(){
    // Generate a probability, start the task timer and set the task
    _taskPc = random(0,100); // NOTE: random num between (min,max-1)
    _taskTimer.start(_taskDuration);
    _LEDTimer.start(0);
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
void TaskManager::set_task(int8_t taskIn){
    if(taskIn != _task_code){
        _task_code = taskIn;
        _task_new_flag = true;
        uint32_t tempDuration = random(_taskDurationMin,_taskDurationMax);

        // Set default duraiton for each task and set the LEDs
        if(_task_code == TASK_TEST){
            _taskDuration = 30000;
            task_LED_test();
        }
        else if(_task_code == TASK_EXPLORE){
            _taskDuration = tempDuration;
            task_LED_explore();
        }
        else if(_task_code == TASK_REST){
            _taskDuration = tempDuration;
        }
        else if(_task_code == TASK_DANCE){
            _taskDuration = _dance_duration;
            task_LED_dance();
        }
        else if(_task_code == TASK_TANTRUM){
            _taskDuration = _tantrum_duration;
            task_LED_collision();
        }
        else if(_task_code == TASK_FINDHUMAN){
            _taskDuration = 20000;
            task_LED_find_human();
        }
        else if(_task_code == TASK_FINDLIGHT){
            _taskDuration = 20000;
            task_LED_find_light();
        }
        else if(_task_code == TASK_FINDDARK){
            _taskDuration = 20000;
            task_LED_find_dark();
        }
        else if(_task_code == TASK_FINDSOUND){
            _taskDuration = 20000;
            task_LED_find_sound();
        }
        else if(_task_code == TASK_INTERACT){
            _taskDuration = 20000;
            task_LED_interact();
        }
        else if(_task_code == TASK_PICKEDUP){
            _taskDuration = 60000;
            // LEDs set based on state in the TaskPickedUp class
        }
        else if(_task_code == TASK_PAUSE){
            _taskDuration = 60000;
            // LEDs set based on state in the TaskPause class
        }
        else if(_task_code == TASK_POUNCE){
            _taskDuration = 20000;
        }
        else{
            _task_code = TASK_EXPLORE;
            _taskDuration = tempDuration;
            task_LED_explore();
        }
        // Start the task timer
        _taskTimer.start(_taskDuration);
    }
}

void TaskManager::set_task_duration(uint32_t taskDur){
    _taskDuration = taskDur;
    _taskTimer.start(_taskDuration);
}

void TaskManager::assign_probability(int8_t moodIn){
    // Moods: [neutral,happy,sad,angry,scared]
    // test is used for forcing task probability
    if(moodIn == MOOD_NEUTRAL){
        _setTaskProb(_taskProbNeutral);
    }
    else if(moodIn == MOOD_HAPPY){
        _setTaskProb(_taskProbHappy);
    }
    else if(moodIn == MOOD_SAD){
        _setTaskProb(_taskProbSad);
    }
    else if(moodIn == MOOD_ANGRY){
        _setTaskProb(_taskProbAngry);
    }
    else if(moodIn == MOOD_SCARED){
        _setTaskProb(_taskProbScared);
    }
    else if(moodIn == MOOD_TEST){
        _setTaskProb(_taskProbTest);
    }
    else{
        _setTaskProb(_taskProbNeutral);
    }
}

//------------------------------------------------------------------------------
// TASK LED FUNCTIONS
void TaskManager::task_LED_collision(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_explore(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 255, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_rest(uint8_t intensity){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(1, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(2, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->show();
}

void TaskManager::task_LED_dance(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_tantrum(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_find_human(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 255));
    _taskLEDs->show();
}

void TaskManager::task_LED_interact(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 255));
    _taskLEDs->show();
}

void TaskManager::task_LED_pickedup_Ok(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_pickedup_panic(){
    if(_LEDTimer.finished()){
        _LEDTimer.start(_LEDOnOffTime);
        _LEDSwitch = !_LEDSwitch;
    }

    if(_LEDSwitch){
        _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
        _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
        _taskLEDs->show();
    }
    else{
        task_LED_off();
    }
}

void TaskManager::task_LED_pause(uint16_t pauseTime){
    if(_LEDTimer.finished()){
        _LEDTimer.start(pauseTime);
    }

    uint8_t LEDVal = _calcRisingLEDVal(pauseTime);
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(LEDVal, LEDVal, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(LEDVal, LEDVal, 0));
    _taskLEDs->show();
}

void TaskManager::task_LED_find_light(){
    uint8_t intens = 255;
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->show();
}

void TaskManager::task_LED_find_dark(){
    uint8_t intens = 85;
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->show();
}

void TaskManager::task_LED_find_sound(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 255));
    _taskLEDs->show();
}

void TaskManager::task_LED_test(){
    if(_LEDTimer.finished()){
        _LEDTimer.start(_LEDOnOffTime);
        _LEDSwitch = !_LEDSwitch;
    }

    if(_LEDSwitch){
        _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
        _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 255));
        _taskLEDs->show();
    }
    else{
        _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 255));
        _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
        _taskLEDs->show();
    }
}

void TaskManager::task_LED_off(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 0));
    _taskLEDs->show();
}

//------------------------------------------------------------------------
// HSV LEDS
void TaskManager::taskLEDHue(uint16_t hue){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->show();
}

void TaskManager::taskLEDHSV(uint16_t hue, uint8_t sat, uint8_t value){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,value)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,value)));
    _taskLEDs->show();
}

void TaskManager::taskLEDCol(uint16_t col){
    uint16_t hue = 65536*col/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->show();
}

void TaskManager::taskLEDCol(uint16_t colL, uint16_t colR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueL)));
    _taskLEDs->show();
}

void TaskManager::taskLEDCSV(uint16_t col, uint8_t sat, uint8_t val){
    uint16_t hue = 65536*col/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,val)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,val)));
    _taskLEDs->show();
}

void TaskManager::taskLEDCSV(uint16_t colL,uint16_t colR,uint8_t satL,uint8_t satR, uint8_t valL, uint8_t valR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueR,satR,valR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueL,satL,valL)));
    _taskLEDs->show();
}
