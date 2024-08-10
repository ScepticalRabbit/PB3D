//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef LASERMANAGER_H
#define LASERMANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PCF8574.h>

#include "Timer.h"
#include "LaserSensor.h"
#include "CollisionDangerFlags.h"
#include "LaserIndex.h"

#define ADDR_GPIO 0x21

// NOTE: The default address for the VL53L0X is 0x29
#define ADDR_LSR_C 0x30
#define ADDR_LSR_UC 0x31
#define ADDR_LSR_DL 0x32
#define ADDR_LSR_DR 0x33
#define ADDR_LSR_HL 0x34
#define ADDR_LSR_HR 0x35
#define ADDR_LSR_L 0x36
#define ADDR_LSR_R 0x37
#define ADDR_LSR_B 0x38


// DEBUG Flag: used to print debugging info to serial on laser status and range
#define DEBUG_LSRMANAGER_DL
#define DEBUG_LSRMANAGER_DR


class LaserManager{
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    LaserManager();

    //--------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //--------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //--------------------------------------------------------------------------
    // Get, set and reset
    int16_t get_range(LaserIndex laser_loc){
        return _laser_ptr_array[laser_loc]->get_range();}

    int8_t get_status(LaserIndex laser_loc){
        return _laser_ptr_array[laser_loc]->get_range_status();}

    uint8_t get_col_code(LaserIndex laser_loc);

    uint8_t getColCodeUC();
    uint8_t getColCodeDL();
    uint8_t getColCodeDR();


private:
    //--------------------------------------------------------------------------
    // HELPER Functions
    //--------------------------------------------------------------------------
    //void _sendByteWithI2C(uint8_t sendAddr, byte sendByte);

    void _update_col_lasers();
    void _update_alt_lasers();
    void _update_updown_lasers();

    DangerFlag _getColCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar);
    DangerFlag _getColCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar,int16_t colSlowDown);
    DangerFlag _getCliffCode(LaserSensor* laser,
                    int16_t cliffClose,int16_t cliffFar);
    DangerFlag _getColCliffCode(LaserSensor* laser,
                    int16_t colClose,int16_t colFar,
                    int16_t cliffClose, int16_t cliffFar);


    //--------------------------------------------------------------------------
    // CLASS VARIABLES
    //--------------------------------------------------------------------------
    // GPIO Expander
    Adafruit_PCF8574 _gpio_expander;

    // Objects for the laser rangers
    LaserSensor _laser_C = LaserSensor(ADDR_LSR_UC,LSR_UP_CENTRE);
    LaserSensor _laser_UC = LaserSensor(ADDR_LSR_C,LSR_CENTRE);
    LaserSensor _laser_DL = LaserSensor(ADDR_LSR_DL,LSR_DOWN_LEFT);
    LaserSensor _laser_DR = LaserSensor(ADDR_LSR_DR,LSR_DOWN_RIGHT);
    LaserSensor _laser_HL = LaserSensor(ADDR_LSR_HL,LSR_HALF_LEFT);
    LaserSensor _laser_HR = LaserSensor(ADDR_LSR_HR,LSR_HALF_RIGHT);
    LaserSensor _laser_L = LaserSensor(ADDR_LSR_L,LSR_LEFT);
    LaserSensor _laser_R = LaserSensor(ADDR_LSR_R,LSR_RIGHT);
    LaserSensor _laser_B = LaserSensor(ADDR_LSR_B,LSR_BACK);

    const static uint8_t _num_lasers = LSR_COUNT;
    LaserSensor* _laser_ptr_array[_num_lasers] = {&_laser_C,
                                                &_laser_UC,
                                                &_laser_DL,
                                                &_laser_DR,
                                                &_laser_HL,
                                                &_laser_HR,
                                                &_laser_L,
                                                &_laser_R,
                                                &_laser_B};

    // LASER ranger variables
    uint16_t _half_body_leng_mm = 80;
    uint16_t _reset_delay = 100;

    int16_t _col_dist_close = _half_body_leng_mm;  // mm
    int16_t _col_dist_far = 120;   // mm
    int16_t _col_dist_slow = 240; // mm
    int16_t _col_dist_lim = 40;    // mm

    int16_t _alt_dist_lim = 0;     // mm
    int16_t _alt_dist_close = 80;  // mm
    int16_t _alt_dist_far = 180;   // mm

    // LSR - UP - DONT CHANGE!!!
    int16_t _up_col_dist_far = 220;    // mm
    int16_t _up_col_dist_close = 180;   // mm
    int16_t _up_col_dist_lim = 40;     // mm
    // LSR- DWN - DONT CHANGE!!!
    int16_t _downCliffDistFar = 170, _downColDistFar = 90;     // mm
    int16_t _downCliffDistClose = 160, _downColDistClose = 70;   // mm
    int16_t _downCliffDistLim = 2000, _downColDistLim = 20;       // mm
    int16_t _downDistCent = 120; // actually measured closer to 125mm

    // LSR - Multi-ranging averaging and error catching
    const static uint8_t _num_measurements = 3;
    int16_t _laser_range_array[_num_lasers][_num_measurements];

    /*
    int16_t _laserRngsL[3] = {0,0,0};
    int16_t _laserRngsR[3] = {0,0,0};
    int16_t _laserRngsU[3] = {0,0,0};
    int16_t _laserRngsD[3] = {0,0,0};
    int8_t _laserStatL[3] = {0,0,0};
    int8_t _laserStatR[3] = {0,0,0};
    int8_t _laserStatU[3] = {0,0,0};
    int8_t _laserStatD[3] = {0,0,0};
    */


    // Timers
    // NOTE: fastest update time on lasers at current setting is 40ms
    uint16_t _col_laser_update_time = 41;
    Timer _col_laser_timer = Timer();
    uint16_t _alt_laser_update_time = 41;
    Timer _alt_laser_timer = Timer();
    uint16_t _updown_laser_update_time = 41;
    Timer _updown_laser_timer = Timer();
};
#endif