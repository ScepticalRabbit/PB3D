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

// Internal libraries
#include "Timer.h"
#include "LaserSensor.h"
#include "CollisionDangerFlags.h"

// Definitions
// Address for digital out 
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif
// Addresses for lasers on I2C
#define ADDR_LSR_L 0x31
#define ADDR_LSR_R 0x32
#define ADDR_LSR_A 0x33
#define ADDR_LSR_U 0x34
#define ADDR_LSR_D 0x35

class LaserManager{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    //---------------------------------------------------------------------------
    LaserManager();

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
    int16_t getRangeL(){return _laserL.getRange();}
    int16_t getRangeR(){return _laserR.getRange();}
    int16_t getRangeA(){return _laserA.getRange();}
    int16_t getRangeU(){return _laserU.getRange();}
    int16_t getRangeD(){return _laserD.getRange();}

    uint8_t getColCodeL();
    uint8_t getColCodeR();
    uint8_t getColCodeU();
    uint8_t getColCodeD();
    uint8_t getColCodeA();

private:
    //---------------------------------------------------------------------------
    // HELPER Functions
    //---------------------------------------------------------------------------
    void _sendByteWithI2C(uint8_t sendAddr, byte sendByte);
    void _updateColLSRs();
    void _updateAltLSR();
    void _updateUpDownLSRs();

    uint8_t _getColCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar);    
    uint8_t _getColCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar,int16_t colSlowDown);    
    uint8_t _getCliffCode(LaserSensor* laser,
                    int16_t cliffClose,int16_t cliffFar);     
    uint8_t _getColCliffCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar,
                    int16_t cliffClose, int16_t cliffFar);
    
    
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    // Objects for the laser rangers
    LaserSensor _laserL = LaserSensor(ADDR_LSR_L,'L');
    LaserSensor _laserR = LaserSensor(ADDR_LSR_R,'R');
    LaserSensor _laserA = LaserSensor(ADDR_LSR_A,'A');
    LaserSensor _laserU = LaserSensor(ADDR_LSR_U,'U');
    LaserSensor _laserD = LaserSensor(ADDR_LSR_D,'D');

    // LASER ranger variables
    uint16_t _halfBodyLengMM = 80;
    uint16_t _resetDelay = 100;
    byte _toSend = B00000000;

    int16_t _colDistClose = _halfBodyLengMM;  // mm
    int16_t _colDistFar = 120;   // mm
    int16_t _colDistSlowD = 240; // mm
    int16_t _colDistLim = 40;    // mm 
    int16_t _altDistLim = 0;     // mm 
    int16_t _altDistClose = 80;  // mm
    int16_t _altDistFar = 180;   // mm

    // LSR - UP - DONT CHANGE!!!
    int16_t _upColDistFar = 220;    // mm
    int16_t _upColDistClose = 180;   // mm
    int16_t _upColDistLim = 40;     // mm  
    // LSR- DWN - DONT CHANGE!!!
    int16_t _downCliffDistFar = 170, _downColDistFar = 90;     // mm
    int16_t _downCliffDistClose = 160, _downColDistClose = 70;   // mm
    int16_t _downCliffDistLim = 2000, _downColDistLim = 20;       // mm
    int16_t _downDistCent = 120; // actually measured closer to 125mm
    
    //bool _collisionLSRFlagU = false, _collisionLSRFlagD = false;

    // Timers
    uint16_t _colLSRUpdateTime = 101;
    Timer _colLSRTimer = Timer();
    uint16_t _altLSRUpdateTime = 101;
    Timer _altLSRTimer = Timer();
    uint16_t _upDownLSRUpdateTime = 101;
    Timer _upDownLSRTimer = Timer();
};
#endif