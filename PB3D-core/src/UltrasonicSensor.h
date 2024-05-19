//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: UltrasonicSensor
//---------------------------------------------------------------------------
/*
The X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>
#include <Wire.h>       // I2C
#include <Ultrasonic.h> // Grove ultrasonic sensor

#include "CollisionDangerFlags.h"

#define COLL_USSENS 7

//---------------------------------------------------------------------------
// CLASS: UltrasonicSensor
//---------------------------------------------------------------------------
class UltrasonicSensor{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    //---------------------------------------------------------------------------
    UltrasonicSensor(){};

    //---------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    //---------------------------------------------------------------------------
    void begin(){};

    //---------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    //---------------------------------------------------------------------------
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset
    //---------------------------------------------------------------------------
    bool getEnabledFlag(){return _isEnabled;}
    void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

    int16_t getRange(){return _range;}
    int16_t getRangeMM(){return _range*10;}

    uint8_t getColCode();

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _isEnabled = true;
    bool _startFlag = true;

    Ultrasonic _ultrasonicRanger = Ultrasonic(COLL_USSENS);

    uint16_t _halfBodyLengMM = 80;
    int16_t _range = 2000;
    int16_t _colDistClose = _halfBodyLengMM/10; // cm
    int16_t _colDistFar = 2*(_halfBodyLengMM/10);  // cm  
    int16_t _colDistSlowD = 3*(_halfBodyLengMM/10); // cm
    int16_t _colDistLim = 4;    // cm 
};
#endif 
