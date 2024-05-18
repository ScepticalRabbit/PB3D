//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CSensorLasers
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include <Adafruit_PCF8574.h>
#include "LaserManager.h"
#include "CollisionDangerFlags.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
LaserManager::LaserManager(){
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void LaserManager::begin(){
    // Start all timers:
    _colLSRTimer.start(0);
    _altLSRTimer.start(0);
    _upDownLSRTimer.start(0);

    // Start the GPIO Board
    if (!_pcf.begin(ADDR_GPIO, &Wire)) {
        Serial.println("LSRMANAGER: Could not init PCF8574");
        while (1);
    }
    else{
        Serial.println("LSRMANAGER: init PCF8574 successful!");
    }
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.pinMode(pp, OUTPUT);
    }

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, LOW);
    }
    delay(_resetDelay);

    // Turn on all sensors - set all high
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, HIGH);
    }
    delay(_resetDelay);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, LOW);
    }
    delay(_resetDelay);

    // Activate first laser sensor
    _pcf.digitalWrite(0, HIGH);
    delay(_resetDelay);
    _laserUC.begin();

    // Activate second laser sensor
    //delay(_resetDelay);
    //_laserDL.begin();

    // Activate third laser sensor
    //delay(_resetDelay);
    //_laserDR.begin();

}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void LaserManager::update(){
    _updateColLSRs();
    _updateAltLSR();
    _updateUpDownLSRs();
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------
uint8_t LaserManager::getColCodeUC(){
    return _getColCode(&_laserUC,_colDistClose,_colDistFar,_colDistSlowD);
}

uint8_t LaserManager::getColCodeDL(){
    return _getColCode(&_laserDL,_colDistClose,_colDistFar,_colDistSlowD);
}

uint8_t LaserManager::getColCodeDR(){
    return _getColCode(&_laserDR,_upColDistClose,_upColDistFar);
}



//---------------------------------------------------------------------------
// HELPER Functions
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void LaserManager::_updateColLSRs(){
    if(!_laserDL.getEnabled() || !_laserDR.getEnabled()){return;}

    if(_colLSRTimer.finished()){
        _colLSRTimer.start(_colLSRUpdateTime);
        _laserDL.startRange();
        _laserDR.startRange();
    }

    if(_laserDL.updateRange()){
        #ifdef DEBUG_LSRMANAGER_L
            Serial.print("LL= "); Serial.print(_laserL.getRangeStatus()); Serial.print(", ");
            Serial.print(_laserL.getRange()); Serial.print(" mm");
            Serial.print(", "); Serial.print(_laserL.getRangeTime()); Serial.println(" ms");
        #endif
    }
    if(_laserDR.updateRange()){
        #ifdef DEBUG_LSRMANAGER_R
            Serial.print("LR= "); Serial.print(_laserR.getRangeStatus()); Serial.print(", ");
            Serial.print(_laserR.getRange()); Serial.print(" mm");
            Serial.print(", "); Serial.print(_laserR.getRangeTime()); Serial.println(" ms");
        #endif
    }
}

//-----------------------------------------------------------------------------
/*
void LaserManager::_updateAltLSR() {
    if(!_laserA.getEnabled()){return;}

    if(_altLSRTimer.finished()){
        _altLSRTimer.start(_altLSRUpdateTime);
        _laserA.startRange();
    }

    if(_laserA.updateRange()){

    }
}
*/
//-----------------------------------------------------------------------------
/*
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
*/
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

