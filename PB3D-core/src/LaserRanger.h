//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef CLASSTEMP_H
#define CLASSTEMP_H

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
    LaserRanger();

    //---------------------------------------------------------------------------
    // BEGIN: called during SETUP
    //---------------------------------------------------------------------------
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during LOOP
    //---------------------------------------------------------------------------
    void update();

    //---------------------------------------------------------------------------
    // DOSOMETHING - called during the main during decision tree 
    //---------------------------------------------------------------------------
    void doSomething();

    //---------------------------------------------------------------------------
    // Get, set and reset
    //---------------------------------------------------------------------------
    bool getEnabledFlag(){return _isEnabled;}
    void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}
    int16_t getRange(){return _range;}

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _isEnabled = true;
    bool _startFlag = true;

    Adafruit_VL53L0X _laserObj = Adafruit_VL53L0X();

    uint8_t _address = 0;
    int16_t _range = 0;
    uint8_t _initNum = 0;
    char _descriptor = 'X';

    bool _collisionFlag = false;
    bool _rangeTimeout = false;
    bool _rangeFlag = false

    int16_t _colDistLim = -1;
    int16_t _colDistClose = -1;
    int16_t _

    
};
#endif 
