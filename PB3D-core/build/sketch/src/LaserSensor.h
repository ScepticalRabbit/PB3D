#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/LaserSensor.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_VL53L0X.h"

#include "PB3DConstants.h"

//---------------------------------------------------------------------------
// LASER RANGER:
//---------------------------------------------------------------------------
class LaserSensor{
public:
     LaserSensor(uint8_t in_addr, ELaserIndex laser_loc){
        _address = in_addr;
        _laser_ind = laser_loc;
    }

    //---------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //---------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void start_range();
    bool update_range();

    //---------------------------------------------------------------------------
    // Get, set and reset
    bool get_enabled(){return _is_enabled;}
    void set_enabled(bool in_flag){_is_enabled = in_flag;}
    int16_t get_range(){return _range;}
    int8_t get_range_status(){return _range_status;}
    uint32_t get_range_time(){return _range_time;}
    char get_location(){return _laser_ind;}
    void set_range_limit(int16_t in_limit){_range_limit = in_limit;}

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    bool _is_enabled = true;
    bool _start_flag = true;

    Adafruit_VL53L0X _laser_obj = Adafruit_VL53L0X();

    const uint16_t _reset_delay = 100;
    uint8_t _address = 0;
    int16_t _range = -1;
    uint8_t _init_num = 0;
    ELaserIndex _laser_ind = LASER_CENTRE;

    bool _range_timeout = false;
    bool _range_flag = false;
    int8_t _range_status = 0;
    uint32_t _range_start_time = 0;
    uint32_t _range_time = 0;
    int16_t _range_limit = 40;
    int16_t _range_max = 2000;
};
#endif
