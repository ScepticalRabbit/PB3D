//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: LaserManager
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
#include <Adafruit_PCF8574.h>

// Internal libraries
#include "Timer.h"
#include "LaserSensor.h"
#include "CollisionDangerFlags.h"

// Definitions
// Address for digital out GPIO
#define ADDR_GPIO 0x21
// Addresses for lasers on I2C
#define ADDR_LSR_UC 0x31
#define ADDR_LSR_DL 0x32
#define ADDR_LSR_DR 0x33


// DEBUG Flag: used to print debugging info to serial on laser status and range
//#define DEBUG_LSRMANAGER_L
//#define DEBUG_LSRMANAGER_R

class LaserManager{
public:
    //---------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    LaserManager();

    //---------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset
     int16_t getRangeUC(){return _laserUC.getRange();}
    int16_t getRangeDL(){return _laserDL.getRange();}
    int16_t getRangeDR(){return _laserDR.getRange();}

    int8_t getStatusUC(){return _laserUC.getRangeStatus();}
    int8_t getStatusDL(){return _laserDL.getRangeStatus();}
    int8_t getStatusDR(){return _laserDR.getRangeStatus();}

    uint8_t getColCodeUC();
    uint8_t getColCodeDL();
    uint8_t getColCodeDR();


private:
    //---------------------------------------------------------------------------
    // HELPER Functions
    //---------------------------------------------------------------------------
    //void _sendByteWithI2C(uint8_t sendAddr, byte sendByte);

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
    // GPIO Expander
    Adafruit_PCF8574 _pcf;

    // Objects for the laser rangers
    LaserSensor _laserUC = LaserSensor(ADDR_LSR_UC,'U');
    LaserSensor _laserDL = LaserSensor(ADDR_LSR_DL,'H');
    LaserSensor _laserDR = LaserSensor(ADDR_LSR_DR,'K');


    // LASER ranger variables
    uint16_t _halfBodyLengMM = 80;
    uint16_t _resetDelay = 100;

    int16_t _colDistClose = _halfBodyLengMM;  // mm
    int16_t _colDistFar = 120;   // mm
    int16_t _colDistSlowD = 240; // mm
    int16_t _colDistLim = 40;    // mm
    int16_t _altDistLim = 0;     // mm
    int16_t _altDistClose = 80;  // mm
    int16_t _altDistFar = 180;   // mm

    // LSR - Multi-ranging averaging and error catching
    int16_t _laserRngsL[3] = {0,0,0};
    int16_t _laserRngsR[3] = {0,0,0};
    int16_t _laserRngsU[3] = {0,0,0};
    int16_t _laserRngsD[3] = {0,0,0};

    int8_t _laserStatL[3] = {0,0,0};
    int8_t _laserStatR[3] = {0,0,0};
    int8_t _laserStatU[3] = {0,0,0};
    int8_t _laserStatD[3] = {0,0,0};

    // LSR - UP - DONT CHANGE!!!
    int16_t _upColDistFar = 220;    // mm
    int16_t _upColDistClose = 180;   // mm
    int16_t _upColDistLim = 40;     // mm
    // LSR- DWN - DONT CHANGE!!!
    int16_t _downCliffDistFar = 170, _downColDistFar = 90;     // mm
    int16_t _downCliffDistClose = 160, _downColDistClose = 70;   // mm
    int16_t _downCliffDistLim = 2000, _downColDistLim = 20;       // mm
    int16_t _downDistCent = 120; // actually measured closer to 125mm

    // Timers
    // NOTE: fastest update time on lasers at current setting is 40ms
    uint16_t _colLSRUpdateTime = 40;
    Timer _colLSRTimer = Timer();
    uint16_t _altLSRUpdateTime = 101;
    Timer _altLSRTimer = Timer();
    uint16_t _upDownLSRUpdateTime = 101;
    Timer _upDownLSRTimer = Timer();
};
#endif