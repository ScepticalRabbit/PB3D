//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "LaserRanger.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
// LaserRanger::LaserRanger(uint8_t inAddr, char inDescript){
//     _address = inAddr;
//     _descriptor = inDescript;
// }

//---------------------------------------------------------------------------
// BEGIN: called during SETUP
//---------------------------------------------------------------------------
void LaserRanger::begin(){
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
void LaserRanger::startRange(){
    if(!_isEnabled){return;}

    _laserObj.startRange();
    _rangeFlag = false;
}

bool LaserRanger::updateRange(){
    if(!_isEnabled){return false;}
    
    if(_laserObj.isRangeComplete() && !_rangeFlag){
        _rangeTimeout = _laserObj.timeoutOccurred();
        if(_rangeTimeout){return false;}

        _range = _laserObj.readRangeResult();
        if(_range <= _rangeLim){_range = -1;}
        _rangeFlag = true;
        return true;
    }
    else{
        return false;
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------



