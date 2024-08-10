#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/LaserManager.h"
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

#include "I2CAddress.h"
#include "Timer.h"
#include "LaserSensor.h"
#include "LaserIndex.h"
#include "CollisionDangerCodes.h"
#include "CollisionStrategy.h"

// DEBUG Flag: used to print debugging info to serial on laser status and range
// #define DEBUG_LSRMANAGER

class LaserManager{
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTOR: pass in pointers to main objects and other sensors
    LaserManager(){};

    //--------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //--------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //--------------------------------------------------------------------------
    // Get, set and reset
    int16_t get_range(ELaserIndex laser_loc){
        return _laser_ptr_array[laser_loc]->get_range();}

    int8_t get_status(ELaserIndex laser_loc){
        return _laser_ptr_array[laser_loc]->get_range_status();}

    EDangerFlag get_collision_code(ELaserIndex _ind);

private:
    //--------------------------------------------------------------------------
    // Laser ranges for collision handling
    uint16_t _half_body_leng_mm = 80;
    uint16_t _reset_delay = 100;

    int16_t _col_dist_close = _half_body_leng_mm;  // mm
    int16_t _col_dist_far = 120;   // mm
    int16_t _col_dist_slow = 240; // mm

    int16_t _alt_dist_lim = 0;     // mm
    int16_t _alt_dist_close = 80;  // mm
    int16_t _alt_dist_far = 180;   // mm

    int16_t _up_col_dist_far = 220;    // mm
    int16_t _up_col_dist_close = 180;   // mm

    int16_t _downCliffDistFar = 170, _downColDistFar = 90;     // mm
    int16_t _downCliffDistClose = 160, _downColDistClose = 70;   // mm
    int16_t _downCliffDistLim = 2000, _downColDistLim = 20;       // mm
    int16_t _downDistCent = 120; // actually measured closer to 125mm
    //--------------------------------------------------------------------------

    // Objects for the laser rangers
    LaserSensor _laser_CC = LaserSensor(ADDR_LSR_UC,LSR_UP_CENTRE);
    LaserSensor _laser_UC = LaserSensor(ADDR_LSR_CC,LSR_CENTRE);
    LaserSensor _laser_DL = LaserSensor(ADDR_LSR_DL,LSR_DOWN_LEFT);
    LaserSensor _laser_DR = LaserSensor(ADDR_LSR_DR,LSR_DOWN_RIGHT);
    LaserSensor _laser_HL = LaserSensor(ADDR_LSR_HL,LSR_HALF_LEFT);
    LaserSensor _laser_HR = LaserSensor(ADDR_LSR_HR,LSR_HALF_RIGHT);
    LaserSensor _laser_LL = LaserSensor(ADDR_LSR_LL,LSR_LEFT);
    LaserSensor _laser_RR = LaserSensor(ADDR_LSR_RR,LSR_RIGHT);
    LaserSensor _laser_BB = LaserSensor(ADDR_LSR_BB,LSR_BACK);
    LaserSensor _laser_AA = LaserSensor(ADDR_LSR_AA,LSR_ALT);

    const static uint8_t _num_lasers = LSR_COUNT;
    LaserSensor* _laser_ptr_array[_num_lasers] = {&_laser_CC,
                                                  &_laser_UC,
                                                  &_laser_DL,
                                                  &_laser_DR,
                                                  &_laser_HL,
                                                  &_laser_HR,
                                                  &_laser_LL,
                                                  &_laser_RR,
                                                  &_laser_BB,
                                                  &_laser_AA};

    // Collision handling strategies for the lasers
    CollisionAvoidSlow _avoid_flat_slow = CollisionAvoidSlow(_col_dist_close,
                                                             _col_dist_far,
                                                             _col_dist_slow);
    CollisionAvoidBasic _avoid_overhead = CollisionAvoidBasic(_up_col_dist_close,
                                                              _up_col_dist_far);
    CollisionCliffAvoid _avoid_cliff = CollisionCliffAvoid(_downColDistClose ,
                                                           _downColDistFar,
                                                           _downCliffDistClose,
                                                           _downCliffDistFar);
    CliffAvoid _avoid_pickup = CliffAvoid(_alt_dist_close,
                                          _alt_dist_far);

    ICollisionStrategy* _laser_strategy_array[_num_lasers] = {&_avoid_flat_slow, // CC
                                                              &_avoid_overhead,  // UC
                                                              &_avoid_cliff,     // DL
                                                              &_avoid_cliff,     // DR
                                                              &_avoid_flat_slow, // HL
                                                              &_avoid_flat_slow, // HR
                                                              &_avoid_flat_slow, // LL
                                                              &_avoid_flat_slow, // RR
                                                              &_avoid_flat_slow, // BB
                                                              &_avoid_pickup,    // AA
                                                              };

    // GPIO Expander
    Adafruit_PCF8574 _gpio_expander;

    uint16_t _laser_update_time = 51;
    Timer _laser_timer = Timer();
};

#endif