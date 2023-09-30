//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: LaserSensor
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "LaserSensor.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void LaserSensor::begin(){
    delay(_resetDelay);
    if(!_laserObj.begin(_address)){
        Serial.print(F("COLLISION: FAILED to init laser "));
        Serial.println(_descriptor);
        _isEnabled = false;
    }
    else{
        Serial.print(F("COLLISION: initialised laser "));
        Serial.println(_descriptor);
        _isEnabled = true;
    }
    delay(_resetDelay);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void LaserSensor::startRange(){
    if(!_isEnabled){return;}

    _laserObj.startRange();
    _rangeStartTime = millis();
    _rangeFlag = false;
}

bool LaserSensor::updateRange(){
    if(!_isEnabled){return false;}
    
    if(_laserObj.isRangeComplete() && !_rangeFlag){
        _rangeTimeout = _laserObj.timeoutOccurred();
        _rangeStatus = _laserObj.readRangeStatus();
        _range = _laserObj.readRangeResult();
        _rangeFlag = true;

        // Check failure conditions for valid ranging, failure range = -1
        if(_rangeTimeout){
            _range = -1;
        }
        if(_rangeStatus != 0){
            _range = -1;
        }
        if(_range <= _rangeLim){
            _range = -1;
        }

        _rangeTime = millis()-_rangeStartTime;
        return true;
    }
    else{
        return false;
    }
}
