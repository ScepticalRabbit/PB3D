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
    else {
        Serial.println("LSRMANAGER: init PCF8574 successful!");
    }
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.pinMode(pp, OUTPUT);
    }

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Turn on all sensors - set all high
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, HIGH);
    }
    delay(_reset_delay);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _pcf.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Activate first laser sensor
    _pcf.digitalWrite(0, HIGH);
    delay(_reset_delay);
    _laserUC.begin();

    // Activate second laser sensor
    _pcf.digitalWrite(1, HIGH);
    delay(_reset_delay);
    _laserDL.begin();

    // Activate third laser sensor
    _pcf.digitalWrite(2, HIGH);
    delay(_reset_delay);
    _laserDR.begin();

}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void LaserManager::update(){
    _updateColLSRs();
    //_updateAltLSR();
    //_updateUpDownLSRs();
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
    if(!_laserDL.get_enabled() || !_laserDR.get_enabled()){return;}

    if(_colLSRTimer.finished()){
        _colLSRTimer.start(_colLSRUpdateTime);
        _laserDL.start_range();
        _laserDR.start_range();
    }

    if(_laserDL.update_range()){
        #ifdef DEBUG_LSRMANAGER_DL
            Serial.print("DL= "); Serial.print(_laserDL.get_range_status()); Serial.print(", ");
            Serial.print(_laserDL.get_range()); Serial.print(" mm");
            Serial.print(", "); Serial.print(_laserDL.get_range_time()); Serial.println(" ms");
        #endif
    }
    if(_laserDR.update_range()){
        #ifdef DEBUG_LSRMANAGER_DR
            Serial.print("DR= "); Serial.print(_laserDR.get_range_status()); Serial.print(", ");
            Serial.print(_laserDR.get_range()); Serial.print(" mm");
            Serial.print(", "); Serial.print(_laserDR.get_range_time()); Serial.println(" ms");
        #endif
    }
}

//-----------------------------------------------------------------------------
/*
void LaserManager::_updateAltLSR() {
    if(!_laserA.get_enabled()){return;}

    if(_altLSRTimer.finished()){
        _altLSRTimer.start(_altLSRUpdateTime);
        _laserA.start_range();
    }

    if(_laserA.update_range()){

    }
}
*/
//-----------------------------------------------------------------------------
/*
void LaserManager::_updateUpDownLSRs(){
    if(!_laserU.get_enabled() || !_laserD.get_enabled()){return;}

    if(_upDownLSRTimer.finished()){
        _upDownLSRTimer.start(_upDownLSRUpdateTime);
        _laserU.start_range();
        _laserD.start_range();
    }

    _laserU.update_range();
    _laserD.update_range();
}
*/
//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,int16_t colSlowDown){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else if(laser->get_range() <= colSlowDown){return DANGER_SLOWD;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getCliffCode(LaserSensor* laser,
        int16_t cliffClose,int16_t cliffFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() >= cliffClose){return DANGER_FAR;}
    else if(laser->get_range() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
uint8_t LaserManager::_getColCliffCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,
        int16_t cliffClose, int16_t cliffFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() >= cliffClose){return DANGER_FAR;}
    else if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else if(laser->get_range() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}

