//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CSensorLasers
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "LaserManager.h"
#include "CollisionDangerFlags.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
LaserManager::LaserManager(){
}

//---------------------------------------------------------------------------
// BEGIN: called during SETUP
//---------------------------------------------------------------------------
void LaserManager::begin(){
    // Start all timers:
    _colLSRTimer.start(0);
    _altLSRTimer.start(0);
    _upDownLSRTimer.start(0);

    // Reset all laser sensors - set all low
    _toSend = B00000000;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);

    // Turn on all sensors - set all high
    _toSend = B00111110;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);

    // Activate first laser sensor
    _toSend = B00000010;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);
    _laserL.begin();

    // Activate second laser sensor
    _toSend = B00000110;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);
    _laserR.begin();
 
    // Activate third laser sensor
    _toSend = B00001110;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);
    _laserA.begin();
    _laserA.setRangeLim(0);

    // Activate fourth laser sensor
    _toSend = B00011110;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);
    _laserU.begin();

    // Activate fifth laser sensor
    _toSend = B00111110;
    _sendByteWithI2C(ADDR_FOLLBOARD,_toSend);
    delay(_resetDelay);
    _laserD.begin();
}

//---------------------------------------------------------------------------
// UPDATE: called during LOOP
//---------------------------------------------------------------------------
void LaserManager::update(){
    _updateColLSRs();
    _updateAltLSR();
    _updateUpDownLSRs();
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------
uint8_t LaserManager::getColCodeL(){
    return _getColCode(&_laserL,_colDistClose,_colDistFar,_colDistSlowD);
}

uint8_t LaserManager::getColCodeR(){
    return _getColCode(&_laserR,_colDistClose,_colDistFar,_colDistSlowD);
}

uint8_t LaserManager::getColCodeU(){
    return _getColCode(&_laserU,_upColDistClose,_upColDistFar);
}

uint8_t LaserManager::getColCodeD(){
    return _getColCliffCode(&_laserD,_downColDistClose,_downColDistFar,
        _downCliffDistClose,_downCliffDistFar);
}

uint8_t LaserManager::getColCodeA(){
    return _getCliffCode(&_laserA,_altDistClose,_altDistFar);
}

//---------------------------------------------------------------------------
// HELPER Functions
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void LaserManager::_sendByteWithI2C(uint8_t sendAddr, byte sendByte){
    Wire.beginTransmission(sendAddr);
    Wire.write(sendByte);
    Wire.endTransmission();
}

//-----------------------------------------------------------------------------
void LaserManager::_updateColLSRs(){
    if(!_laserL.getEnabled() || !_laserR.getEnabled()){return;}

    if(_colLSRTimer.finished()){
        _colLSRTimer.start(_colLSRUpdateTime);
        _laserL.startRange();
        _laserR.startRange();
    }

    if(_laserL.updateRange()){

    }
    if(_laserR.updateRange()){
        
    }
}

//-----------------------------------------------------------------------------
void LaserManager::_updateAltLSR() {
    if(!_laserA.getEnabled()){return;}

    if(_altLSRTimer.finished()){
        _altLSRTimer.start(_altLSRUpdateTime);
        _laserA.startRange();
    }
    
    if(_laserA.updateRange()){

    }
}

//-----------------------------------------------------------------------------
void LaserManager::_updateUpDownLSRs(){
    if(!_laserU.getEnabled() || !_laserD.getEnabled()){return;}

    if(_upDownLSRTimer.finished()){
        _upDownLSRTimer.start(_upDownLSRUpdateTime);
        _laserU.startRange();
        _laserD.startRange();
    }

    _laserU.updateRange();
    _laserD.updateRange();
}

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar){

    if(laser->getRange() < 0 ){return DANGER_NONE;}

    if(laser->getRange() <= colClose){return DANGER_CLOSE;}
    else if(laser->getRange() <= colFar){return DANGER_FAR;}
    else{return DANGER_NONE;}       
}   

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,int16_t colSlowDown){

    if(laser->getRange() < 0 ){return DANGER_NONE;}

    if(laser->getRange() <= colClose){return DANGER_CLOSE;}
    else if(laser->getRange() <= colFar){return DANGER_FAR;}
    else if(laser->getRange() <= colSlowDown){return DANGER_SLOWD;} 
    else{return DANGER_NONE;}       
} 

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getCliffCode(LaserSensor* laser,
        int16_t cliffClose,int16_t cliffFar){
    
    if(laser->getRange() < 0 ){return DANGER_NONE;}

    if(laser->getRange() >= cliffClose){return DANGER_FAR;}
    else if(laser->getRange() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}       
}   

//----------------------------------------------------------------------------- 
uint8_t LaserManager::_getColCliffCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,
        int16_t cliffClose, int16_t cliffFar){
    
    if(laser->getRange() < 0 ){return DANGER_NONE;}

    if(laser->getRange() >= cliffClose){return DANGER_FAR;}
    else if(laser->getRange() <= colClose){return DANGER_CLOSE;}
    else if(laser->getRange() <= colFar){return DANGER_FAR;}
    else if(laser->getRange() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}
    
