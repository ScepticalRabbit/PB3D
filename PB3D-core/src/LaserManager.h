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

#include <PB3DI2CAddresses.h>
#include <PB3DConstants.h>

#include "PB3DTimer.h"
#include "LaserSensor.h"
#include "CollisionStrategy.h"

// DEBUG Flag: used to print debugging info to serial on laser status and range
// #define DEBUG_LSRMANAGER

class LaserManager{
public:
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
    int16_t get_range(uint8_t laser_loc){
        return _laser_ptr_array[laser_loc]->get_range();}

    int8_t get_status(ELaserIndex laser_loc){
        return _laser_ptr_array[laser_loc]->get_range_status();}
    int8_t get_status(uint8_t laser_loc){
        return _laser_ptr_array[laser_loc]->get_range_status();}

    EDangerCode get_collision_code(ELaserIndex _ind);

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
    LaserSensor _laser_CC = LaserSensor(ADDR_LSR_UC,LASER_UP_CENTRE);
    LaserSensor _laser_UC = LaserSensor(ADDR_LSR_CC,LASER_CENTRE);
    LaserSensor _laser_DL = LaserSensor(ADDR_LSR_DL,LASER_DOWN_LEFT);
    LaserSensor _laser_DR = LaserSensor(ADDR_LSR_DR,LASER_DOWN_RIGHT);
    LaserSensor _laser_HL = LaserSensor(ADDR_LSR_HL,LASER_HALF_LEFT);
    LaserSensor _laser_HR = LaserSensor(ADDR_LSR_HR,LASER_HALF_RIGHT);
    LaserSensor _laser_LL = LaserSensor(ADDR_LSR_LL,LASER_LEFT);
    LaserSensor _laser_RR = LaserSensor(ADDR_LSR_RR,LASER_RIGHT);
    LaserSensor _laser_BB = LaserSensor(ADDR_LSR_BB,LASER_BACK);
    LaserSensor _laser_AA = LaserSensor(ADDR_LSR_AA,LASER_ALT);

    const static uint8_t _num_lasers = LASER_COUNT;
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
    CollisionAvoidBasic _avoid_basic = CollisionAvoidBasic(_col_dist_close,
                                                           _col_dist_far);

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

    ECollisionStrategy _laser_strategy_array[_num_lasers] = {AVOID_FLAT_SLOW, // CC
                                                             AVOID_OVERHEAD,  // UC
                                                             AVOID_CLIFF,     // DL
                                                             AVOID_CLIFF,     // DR
                                                             AVOID_FLAT_SLOW, // HL
                                                             AVOID_FLAT_SLOW, // HR
                                                             AVOID_BASIC, // LL
                                                             AVOID_BASIC, // RR
                                                             AVOID_FLAT_SLOW, // BB
                                                             AVOID_PICKUP,    // AA
                                                            };

    // GPIO Expander
    Adafruit_PCF8574 _gpio_expander;

    uint16_t _laser_update_time = 51;
    Timer _laser_timer = Timer();
};

#endif