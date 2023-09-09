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
// BEGIN: called during SETUP
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
// UPDATE: called during LOOP
//---------------------------------------------------------------------------
void LaserSensor::startRange(){
    if(!_isEnabled){return;}

    _laserObj.startRange();
    _rangeFlag = false;
}

bool LaserSensor::updateRange(){
    if(!_isEnabled){return false;}
    
    if(_laserObj.isRangeComplete() && !_rangeFlag){
        _rangeTimeout = _laserObj.timeoutOccurred();
        if(_rangeTimeout){return false;}

        _range = _laserObj.readRangeResult();
        if(_range <= _rangeLim){_range = _rangeLim;}
        _rangeFlag = true;
        return true;
    }
    else{
        return false;
    }
}
