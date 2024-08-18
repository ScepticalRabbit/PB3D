#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoodManager.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoodManager.h"

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void MoodManager::begin(){
    // Generate a probability, start the mood timer and set the mood
    _mood_percent = random(0,100); // NOTE: random num between (min,max-1)
    set_mood(MOOD_NEUTRAL);
    _mood_timer.start(0);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void MoodManager::update(){
    if(!_enabled){return;}

    if(_mood_timer.finished()){
        int8_t prob = random(0,100);

        if(prob <= 10){
            dec_mood_score();
        }
        else if(prob >= 90){
            inc_mood_score();
        }

        _mood_percent = random(0,100)+_mood_score;
        _mood_percent = constrain(_mood_percent,0,100);

        if((_mood_percent >= 0) && (_mood_percent < _mood_prob_array[0])){ // NEGATIVE MOOD
            _mood_neg_percent = random(0,100)+_mood_score;

            if((_mood_neg_percent >= 0) && (_mood_neg_percent <= _mood_prob_array[0])){
                set_mood(MOOD_SAD);
            }
            else if((_mood_percent >= _mood_prob_array[1]) && (_mood_percent <= _mood_prob_array[2])){
                set_mood(MOOD_ANGRY);
            }
            else if((_mood_percent >= _mood_prob_array[2]) && (_mood_percent <= _mood_prob_array[3])){
                set_mood(MOOD_SCARED);
            }
            else{
                set_mood(MOOD_SAD);
            }
        }
        else if((_mood_percent >= _mood_prob_array[0]) && (_mood_percent <= _mood_prob_array[1])){ // NEUTRAL MOOD
            set_mood(MOOD_NEUTRAL);
        }
        else if((_mood_percent >= _mood_prob_array[1]) && (_mood_percent <= _mood_prob_array[2])){ // POSITIVE MOOD
            set_mood(MOOD_HAPPY);
        }
        else{ // DEFAULT NEUTRAL
            set_mood(MOOD_NEUTRAL);
        }
    }
}

//------------------------------------------------------------------------------
// Get, set and reset
void MoodManager::set_mood(EMoodCode mood){
    if(mood != _mood_code){
        _set_mood(mood);

        _mood_new_flag = true;
        _mood_duration = random(_mood_min_duration,_mood_max_duration);
        _mood_timer.start(_mood_duration);
    }
}

//------------------------------------------------------------------------------
// Mood Score Functions
void MoodManager::mod_mood_score(int8_t modifier){
    int16_t tempScore = _mood_score+modifier;
    tempScore = constrain(tempScore,_mood_score_min,_mood_score_max);
    _mood_score = tempScore;
}

void MoodManager::inc_mood_score(){
    int16_t tempScore = _mood_score+_mood_score_increment;
    tempScore = constrain(tempScore,_mood_score_min,_mood_score_max);
    _mood_score = tempScore;
}

void MoodManager::dec_mood_score(){
    int16_t tempScore = _mood_score-_mood_score_increment;
    tempScore = constrain(tempScore,_mood_score_min,_mood_score_max);
    _mood_score = tempScore;
}

//------------------------------------------------------------------------------
// Mood LED Functions

void MoodManager::mood_LED_neutral(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->show();
}

void MoodManager::mood_LED_happy(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->show();
}

void MoodManager::mood_LED_angry(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->show();
}

void MoodManager::mood_LED_sad(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->show();
}

void MoodManager::mood_LED_scared(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->show();
}

void MoodManager::mood_LED_curious(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->show();
}

void MoodManager::mood_LED_test(){
    uint8_t ledVal = 205+_mood_score;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->show();
}

void MoodManager::mood_LED_off(){
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->show();
}


void MoodManager::_set_mood(EMoodCode mood){

    switch(mood){
        case MOOD_NEUTRAL:
            _mood_code = MOOD_NEUTRAL;
            mood_LED_neutral();
            break;

        case MOOD_HAPPY:
            _mood_code = MOOD_HAPPY;
            mood_LED_happy();
            break;

        case MOOD_SAD:
            _mood_code = MOOD_SAD;
            mood_LED_sad();
            break;

        case MOOD_ANGRY:
            _mood_code = MOOD_ANGRY;
            mood_LED_angry();
            break;

        case MOOD_SCARED:
            _mood_code = MOOD_SCARED;
            mood_LED_scared();
            break;

        case MOOD_TEST:
            _mood_code = MOOD_TEST;
            mood_LED_test();
            break;

        default:
            _mood_code = MOOD_NEUTRAL;
            mood_LED_neutral();
            break;
    }

}
