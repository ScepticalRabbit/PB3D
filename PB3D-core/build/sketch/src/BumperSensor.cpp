#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/BumperSensor.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "BumperSensor.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
BumperSensor::BumperSensor(){

}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void BumperSensor::begin(){
    // TODO: check that there is something at this address 
    //Wire.requestFrom(ADDR_BUMPERS,1);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void BumperSensor::update(){
    if(!_isEnabled){return;}

    // Request a byte worth of digital pins from the follower Xiao
    Wire.requestFrom(ADDR_BUMPERS,1);
    // Read a byte from the follower
    byte bumperByte = B00000000;
    while(Wire.available()){
        bumperByte  = Wire.read();   
    }
    _bumperReadByte = bumperByte;

    if((_bumperReadByte & _bumperBytes[_bumpLeft]) == _bumperBytes[_bumpLeft]){
        _bumperFlags[_bumpLeft] = true;
    }
    if ((_bumperReadByte & _bumperBytes[_bumpRight]) == _bumperBytes[_bumpRight]){
        _bumperFlags[_bumpRight] = true;
    }

    // Loop over bumper flags to see if any are tripped
    for(uint8_t ii=0; ii<_numBumpers; ii++){
        if(_bumperFlags[ii]){
            _bumperAnyFlag = true;
        }
    }

    // If the bumpers are hit too many times decrease mood
    if(_bumperAnyFlag){
        _bumpCount++;
    }
}

//---------------------------------------------------------------------------
uint8_t BumperSensor::getColCode(uint8_t bumpCode){
    if(bumpCode >= _numBumpers){
        return DANGER_NONE;
    }
    if(_bumperFlags[bumpCode]){
        return DANGER_CLOSE;
    }
    else{
        return DANGER_NONE;
    }
}

//---------------------------------------------------------------------------
void BumperSensor::reset(){
    _bumperAnyFlag = false;
    for(uint8_t ii=0; ii<_numBumpers; ii++){
        _bumperFlags[ii] = false;    
    }
}
