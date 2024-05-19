//---------------------------------------------------------------------------
// PET BOT - PB3D!
// CLASS: LaserRanger
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <Arduino.h>
#include <Wire.h> // I2C

#include "Adafruit_VL53L0X.h"

//---------------------------------------------------------------------------
// LASER RANGER:
//---------------------------------------------------------------------------
class LaserSensor{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
     LaserSensor(uint8_t inAddr, char inDescript){
        _address = inAddr;
        _descriptor = inDescript;
    }

    //---------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void startRange();
    bool updateRange();

    //---------------------------------------------------------------------------
    // Get, set and reset
    bool getEnabled(){return _isEnabled;}
    void setEnabled(bool inFlag){_isEnabled = inFlag;}
    int16_t getRange(){return _range;}
    int8_t getRangeStatus(){return _rangeStatus;}
    uint32_t getRangeTime(){return _rangeTime;}
    char getDescriptor(){return _descriptor;}
    void setRangeLim(int16_t inLim){_rangeLim = inLim;}

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    bool _isEnabled = true;
    bool _startFlag = true;

    Adafruit_VL53L0X _laserObj = Adafruit_VL53L0X();

    const uint16_t _resetDelay = 100;
    uint8_t _address = 0;
    int16_t _range = -1;
    uint8_t _initNum = 0;
    char _descriptor = 'X';


    bool _rangeTimeout = false;
    bool _rangeFlag = false;
    int8_t _rangeStatus = 0;
    uint32_t _rangeStartTime = 0;
    uint32_t _rangeTime = 0;
    int16_t _rangeLim = 40;
    int16_t _rangeMax = 2000;
};
#endif
