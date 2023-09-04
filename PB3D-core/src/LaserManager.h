//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: 
//---------------------------------------------------------------------------
/*
The X class is part of the PetBot (PB) program. 

Author: Lloyd Fletcher
*/
#ifndef LASERMANAGER_H
#define LASERMANAGER_H

// Core libraries
#include <Arduino.h>
#include <Wire.h>
// External libraires
#include "Adafruit_VL53L0X.h"
// Internal libraries
#include "Timer.h"

class LaserManager{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    //---------------------------------------------------------------------------
    LaserManager();

    //---------------------------------------------------------------------------
    // BEGIN: called during SETUP
    //---------------------------------------------------------------------------
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during LOOP
    //---------------------------------------------------------------------------
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset
    //---------------------------------------------------------------------------

private:
    //---------------------------------------------------------------------------
    // LASER INIT: set all I2C addresses
    //---------------------------------------------------------------------------
    void _initLSR(byte sendByte, Adafruit_VL53L0X* LSRObj, bool* LSROn,
                    uint8_t LSRAddr,char LSRStr);
    void _setLSRAddrs();
    
    
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    // Objects for the laser ranger vl53l0x
    Adafruit_VL53L0X _laserL = Adafruit_VL53L0X();
    Adafruit_VL53L0X _laserR = Adafruit_VL53L0X();
    Adafruit_VL53L0X _laserA = Adafruit_VL53L0X();
    Adafruit_VL53L0X _laserU = Adafruit_VL53L0X();
    Adafruit_VL53L0X _laserD = Adafruit_VL53L0X();

    // LASER ranger variables
    uint16_t _halfBodyLengMM = 80;
    uint16_t _resetDelay = 100;
    int16_t _LSRColDistClose = _halfBodyLengMM;  // mm
    int16_t _LSRColDistFar = 120;   // mm
    int16_t _LSRColDistSlowD = 240; // mm
    int16_t _LSRColDistLim = 40;    // mm  
    int16_t _LSRAltDist = 80;       // mm
    bool _collisionLSRFlagL = false;
    bool _collisionLSRFlagR = false;
    bool _collisionLSRFlagB = false;

    // LSR - UP - DONT CHANGE!!!
    int16_t _LSRUpDistFar = 220;    // mm
    int16_t _LSRUpDistClose = 180;   // mm
    int16_t _LSRUpDistLim = 40;     // mm  
    // LSR- DWN - DONT CHANGE!!!
    int16_t _LSRDownCliffDistFar = 170, _LSRDownColDistFar = 90;     // mm
    int16_t _LSRDownCliffDistClose = 160, _LSRDownColDistClose = 70;   // mm
    int16_t _LSRDownCliffDistLim = 2000, _LSRDownColDistLim = 20;       // mm
    int16_t _LSRDownDistCent = 120; // actually measured closer to 145mm
    bool _collisionLSRFlagU = false, _collisionLSRFlagD = false;

    // Actual range values in mm
    bool _LSRLOn = false, _LSRROn = false, _LSRAOn = false;
    int16_t _LSRRangeL=0, _LSRRangeR=0, _LSRRangeA=0; //mm
    bool _LSRFlagL=false, _LSRFlagR=false, _LSRFlagA=false;
    bool _LSRL_TO=false, _LSRR_TO=false, _LSRA_TO=false;

    bool _LSRUOn = false, _LSRDOn = false;
    int16_t _LSRRangeU=0, _LSRRangeD=0; //mm
    bool _LSRFlagU=false, _LSRFlagD=false;
    bool _LSRU_TO=false, _LSRD_TO=false;

    uint16_t _colLSRUpdateTime = 101;
    Timer _colLSRTimer = Timer();
    uint16_t _altLSRUpdateTime = 101;
    Timer _altLSRTimer = Timer();
    uint16_t _upDownLSRUpdateTime = 101;
    Timer _upDownLSRTimer = Timer();
};
#endif