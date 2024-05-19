//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: BumperSensor
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef BUMPERSENSOR_H
#define BUMPERSENSOR_H

#include <Arduino.h>
#include <Wire.h> // I2C

#include "Timer.h"
#include "CollisionDangerFlags.h"

#define ADDR_BUMPERS 9

#define BUMP_NUM = 2
#define BUMP_LEFT = 0
#define BUMP_RIGHT = 1

//---------------------------------------------------------------------------
// CLASS TEMPLATE: 
//---------------------------------------------------------------------------
class BumperSensor{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    //---------------------------------------------------------------------------
    BumperSensor();

    //---------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    //---------------------------------------------------------------------------
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    //---------------------------------------------------------------------------
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset
    //---------------------------------------------------------------------------
    bool getEnabledFlag(){return _isEnabled;}
    void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

    bool getBumpFlag(){return _bumperAnyFlag;}
    bool getBumpThresCheck(){return (_bumpCount >= _bumpThres);}
    int8_t getBumpCount(){return _bumpCount;}
    void resetBumpCount(){_bumpCount= 0;}

    uint8_t getColCode(uint8_t bumpCode);
    void reset();

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _isEnabled = true;
    bool _startFlag = true;

    const static uint8_t _numBumpers = 2;
    const uint8_t _bumpLeft = 0;
    const uint8_t _bumpRight = 1;

    byte _bumperReadByte = B00000000;
    bool _bumperAnyFlag = false;
    bool _bumperFlags[_numBumpers] = {false,false};
    byte _bumperBytes[_numBumpers] = {B00000001,B00000010};

    int8_t _bumpCount = 0;
    int8_t _bumpThres = 13;
};
#endif 
