#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Encoder.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: Encoder
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "Encoder.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR
Encoder::Encoder(int8_t pinA, int8_t pinB){    
    _pinA = pinA;
    _pinB = pinB;
    _encCountCurr = 0;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void Encoder::begin(){
    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);
    _speedTimer.start(0);
    _speedFiltMMPS.begin();
    _speedFiltCPS.begin();
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
// Update for the right encoder '!='
void Encoder::updateNEQ(){
    if(digitalRead(_pinA) != digitalRead(_pinB)){
        _encCountCurr++;
        _encDirCode = 'F';
    }
    else{
        _encCountCurr--;
        _encDirCode = 'B';
    } 
}

// Update for the left encoder '=='
void Encoder::updateEQ(){
    if(digitalRead(_pinA) == digitalRead(_pinB)){
        _encCountCurr++;
        _encDirCode = 'F';
    }
    else{
        _encCountCurr--;
        _encDirCode = 'B';
    } 
}

//-------------------------------------------------------------------------
// SPEED CALCULATION FUNCTIONS
void Encoder::updateSpeed(){
    if(_speedTimer.finished()){
        // Get the current time and restart the timer
        double timeIntS = double(_speedTimer.getTime())/1000.0;
        _speedTimer.start(_speedUpdateTime);
        
        // Calculate distance travelled using the encoder count difference
        double distMM = double(_encCountCurr-_encCountPrevForSpeed)*_mmPerCount;

        // Calculate speed in 'counts/second'
        _rawSpeedCPS = double(_encCountCurr-_encCountPrevForSpeed)/timeIntS;
        // Smooth the raw speed using the filter
        _smoothSpeedCPS = _speedFiltCPS.filter(_rawSpeedCPS); 
        
        // Calculate raw and smoothed speed
        _rawSpeedMMPS = distMM/timeIntS;
        // Smooth the raw speed using the filter
        _smoothSpeedMMPS = _speedFiltMMPS.filter(_rawSpeedMMPS); 

        // Store the encoder count for the next round of calculations
        _encCountPrevForSpeed = _encCountCurr;
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void Encoder::resetFilter(){
    _speedFiltMMPS.reset();
    _speedFiltCPS.reset();
}

void Encoder::resetFilter(float inVal){
    _speedFiltMMPS.reset(inVal);
    _speedFiltCPS.reset(inVal);
}