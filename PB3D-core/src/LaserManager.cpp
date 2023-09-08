//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CSensorLasers
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "LaserManager.h"
#include "CollisionFlags.h"

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
    return _getColCode(&_laserD,_downColDistClose,_downColDistFar,
        _downCliffDistClose,_downCliffDistFar);
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
    _laserA.updateRange();
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
uint8_t LaserManager::_getColCode(LaserRanger* laser,
        int16_t colClose,int16_t colFar){

    if(laser->getRange() <= colClose){return COLL_CLOSE;}
    else if(laser->getRange() <= colFar){return COLL_FAR;}
    else{return 0;}       
}   

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCode(LaserRanger* laser,
        int16_t colClose,int16_t colFar,int16_t colSlowDown){

    if(laser->getRange() <= colClose){return COLL_CLOSE;}
    else if(laser->getRange() <= colFar){return COLL_FAR;}
    else if(laser->getRange() <= colSlowDown){return COLL_SLOWD;} 
    else{return 0;}       
}   

//----------------------------------------------------------------------------- 
uint8_t LaserManager::_getColCode(LaserRanger* laser,
        int16_t colClose,int16_t colFar,
        int16_t cliffClose, int16_t cliffFar){
    
    if(laser->getRange() >= cliffClose){return COLL_CLOSE;}
    else if(laser->getRange() <= colClose){return COLL_CLOSE;}
    else if(laser->getRange() <= colFar){return COLL_FAR;}
    else if(laser->getRange() >= cliffFar){return COLL_FAR;}
    else{return 0;}
}
    