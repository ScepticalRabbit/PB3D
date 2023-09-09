//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: LaserRanger
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef LASERRANFER_H
#define LASERRANFER_H

#include <Arduino.h>
#include <Wire.h> // I2C

#include "Adafruit_VL53L0X.h"

//---------------------------------------------------------------------------
// LASER RANGER: 
//---------------------------------------------------------------------------
class LaserRanger{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    //---------------------------------------------------------------------------
    LaserRanger(uint8_t inAddr, char inDescript){
        _address = inAddr;
        _descriptor = inDescript;
    }

    //---------------------------------------------------------------------------
    // BEGIN: called during SETUP
    //---------------------------------------------------------------------------
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during LOOP
    //---------------------------------------------------------------------------
    void startRange();
    bool updateRange();

    //---------------------------------------------------------------------------
    // Get, set and reset
    //---------------------------------------------------------------------------
    bool getEnabled(){return _isEnabled;}
    void setEnabled(bool inFlag){_isEnabled = inFlag;}
    int16_t getRange(){return _range;}
    char getDescriptor(){return _descriptor;}

private:
    //---------------------------------------------------------------------------
    // Helper Functions
    //---------------------------------------------------------------------------
    

    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _isEnabled = true;
    bool _startFlag = true;

    Adafruit_VL53L0X _laserObj = Adafruit_VL53L0X();

    const uint16_t _resetDelay = 100;
    uint8_t _address = 0;
    int16_t _range = 0;
    uint8_t _initNum = 0;
    char _descriptor = 'X';


    bool _rangeTimeout = false;
    bool _rangeFlag = false;
    int16_t _rangeLim = 40;

    // All values below in millimeters for consistency with laser ranger
    /*
    bool _collisionFlag = false;
    
    int16_t _colDistClose = -1;
    int16_t _colDistFar = -1;
    int16_t _colDistSlowD = -1;
    
    bool _cliffDetectOn = false;
    bool _cliffFlag = false;
    int16_t _cliffDistLim = 2000;
    int16_t _cliffDistClose = -1;
    int16_t _cliffDistFar = -1;  
    */  
};
#endif 
