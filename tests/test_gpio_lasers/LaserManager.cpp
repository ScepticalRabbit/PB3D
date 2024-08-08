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
    _col_laser_timer.start(0);
    _alt_laser_timer.start(0);
    _updown_laser_timer.start(0);

    // Start the GPIO Board
    if (!_gpio_expander.begin(ADDR_GPIO, &Wire)) {
        Serial.println("LSRMANAGER: Could not init PCF8574");
        while (1);
    }
    else {
        Serial.println("LSRMANAGER: init PCF8574 successful!");
    }
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.pinMode(pp, OUTPUT);
    }

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Turn on all sensors - set all high
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, HIGH);
    }
    delay(_reset_delay);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Activate ALL lasers one by one
    for (uint8_t rr=0; rr<_num_lasers; rr++) {
        if (rr < 8){
            _gpio_expander.digitalWrite(rr, HIGH);
        }

        _laser_ptr_array[rr]->begin();

        for (uint8_t mm=0; mm<_num_measurements; mm++) {
            _laser_range_array[rr][mm] = 0;
        }
    }
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void LaserManager::update(){
    _update_col_lasers();
    _update_alt_lasers();
    _update_updown_lasers();
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------
uint8_t LaserManager::getColCodeUC(){
    return _getColCode(&_laserUC,_col_dist_close,_col_dist_far,_col_dist_slow);
}

uint8_t LaserManager::getColCodeDL(){
    return _getColCode(&_laserDL,_col_dist_close,_col_dist_far,_col_dist_slow);
}

uint8_t LaserManager::getColCodeDR(){
    return _getColCode(&_laserDR,_up_col_dist_close,_up_col_dist_far);
}



//---------------------------------------------------------------------------
// HELPER Functions
//---------------------------------------------------------------------------

//-----------------------------------------------------------------------------
void LaserManager::_update_col_lasers(){

    if(!_laserDL.get_enabled() || !_laserDR.get_enabled()){return;}

    if(_col_laser_timer.finished()){
        _col_laser_timer.start(_col_laser_update_time);
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
void LaserManager::_update_alt_lasers() {
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
void LaserManager::_update_updown_lasers(){
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
DangerFlag LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
DangerFlag LaserManager::_getColCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,int16_t colSlowDown){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else if(laser->get_range() <= colSlowDown){return DANGER_SLOW;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
DangerFlag LaserManager::_getCliffCode(LaserSensor* laser,
        int16_t cliffClose,int16_t cliffFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() >= cliffClose){return DANGER_FAR;}
    else if(laser->get_range() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}

//-----------------------------------------------------------------------------
DangerFlag LaserManager::_getColCliffCode(LaserSensor* laser,
        int16_t colClose,int16_t colFar,
        int16_t cliffClose, int16_t cliffFar){

    if(laser->get_range() < 0 ){return DANGER_NONE;}

    if(laser->get_range() >= cliffClose){return DANGER_FAR;}
    else if(laser->get_range() <= colClose){return DANGER_CLOSE;}
    else if(laser->get_range() <= colFar){return DANGER_FAR;}
    else if(laser->get_range() >= cliffFar){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}

