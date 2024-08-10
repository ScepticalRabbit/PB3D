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

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
// MoodManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){_moodLEDs = RGBLEDs;}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void MoodManager::begin(){
    // Generate a probability, start the mood timer and set the mood
    _moodPc = random(0,100); // NOTE: random num between (min,max-1)
    setMood(_moodPc);
    _moodTimer.start(_moodDuration);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void MoodManager::update(){
    if(!_isEnabled){return;}
    if(_moodTimer.finished()){
        // Randomly update mood score before generating new mood
        int8_t prob = random(0,100);
        if(prob <= 10){decMoodScore();}
        else if(prob >= 90){incMoodScore();}

        // Generate a new mood probability
        _moodPc = random(0,100)+_moodScore; // NOTE: random num between (min,max-1)
        _moodPc = constrain(_moodPc,0,100);

        // Determine where on the happy/sad axis
        if((_moodPc >= 0) && (_moodPc < _moodProbVec[0])){ // NEGATIVE MOOD
        _moodNegPc = random(0,100)+_moodScore;

            if((_moodNegPc >= 0) && (_moodNegPc <= _moodProbVec[0])){
                setMood(MOOD_SAD);
            }
            else if((_moodPc >= _moodProbVec[1]) && (_moodPc <= _moodProbVec[2])){
                setMood(MOOD_ANGRY);
            }
            else if((_moodPc >= _moodProbVec[2]) && (_moodPc <= _moodProbVec[3])){
                setMood(MOOD_SCARED);
            }
            else{
                setMood(MOOD_SAD);
            }
        }
        else if((_moodPc >= _moodProbVec[0]) && (_moodPc <= _moodProbVec[1])){ // NEUTRAL MOOD
            setMood(MOOD_NEUTRAL);
        }
        else if((_moodPc >= _moodProbVec[1]) && (_moodPc <= _moodProbVec[2])){ // POSITIVE MOOD
            setMood(MOOD_HAPPY);
        }
        else{ // DEFAULT NEUTRAL
            setMood(MOOD_NEUTRAL);
        }
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------
void MoodManager::setMood(int16_t moodIn){
    if(moodIn != _moodCode){
        _setMood(moodIn);

        _moodNewFlag = true;
        _moodDuration = random(_moodMinDuration,_moodMaxDuration);
        _moodTimer.start(_moodDuration);
    }
}

//---------------------------------------------------------------------------
// Mood Score Functions
//---------------------------------------------------------------------------
void MoodManager::modMoodScore(int8_t modifier){
    int16_t tempScore = _moodScore+modifier;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
}

void MoodManager::incMoodScore(){
    int16_t tempScore = _moodScore+_moodScoreInc;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
}

void MoodManager::decMoodScore(){
    int16_t tempScore = _moodScore-_moodScoreInc;
    tempScore = constrain(tempScore,_moodScoreMin,_moodScoreMax);
    _moodScore = tempScore;
}

//---------------------------------------------------------------------------
// Mood LED Functions
//---------------------------------------------------------------------------

void MoodManager::moodLEDNeutral(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, 0));
    _moodLEDs->show();
}

void MoodManager::moodLEDHappy(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, 0));
    _moodLEDs->show();
}

void MoodManager::moodLEDAngry(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, 0));
    _moodLEDs->show();
}

void MoodManager::moodLEDSad(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, ledVal));
    _moodLEDs->show();
}

void MoodManager::moodLEDScared(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, ledVal, ledVal));
    _moodLEDs->show();
}

void MoodManager::moodLEDCurious(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, 0, ledVal));
    _moodLEDs->show();
}

void MoodManager::moodLEDTest(){
    uint8_t ledVal = 205+_moodScore;
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(ledVal, ledVal, ledVal));
    _moodLEDs->show();
}

void MoodManager::moodLEDOff(){
    _moodLEDs->setPixelColor(1, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->setPixelColor(2, _moodLEDs->Color(0, 0, 0));
    _moodLEDs->show();
}

//---------------------------------------------------------------------------
// Helper Functions
//---------------------------------------------------------------------------
void MoodManager::_setMood(int8_t moodIn){
    if(moodIn == MOOD_NEUTRAL){
        _moodCode = MOOD_NEUTRAL;
        moodLEDNeutral();
    }
    else if(moodIn == MOOD_HAPPY){
        _moodCode = MOOD_HAPPY;
        moodLEDHappy();
    }
    else if(moodIn == MOOD_SAD){
        _moodCode = MOOD_SAD;
        moodLEDSad();
    }
    else if(moodIn == MOOD_ANGRY){
        _moodCode = MOOD_ANGRY;
        moodLEDAngry();
    }
    else if(moodIn == MOOD_SCARED){
        _moodCode = MOOD_SCARED;
        moodLEDScared();
    }
    else if(moodIn == MOOD_TEST){
        _moodCode = MOOD_TEST;
        moodLEDTest();
    }
    else{
        _moodCode = MOOD_NEUTRAL;
        moodLEDNeutral();
    }
}
