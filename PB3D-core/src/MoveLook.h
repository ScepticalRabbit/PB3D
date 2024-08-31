//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_LOOK_H
#define MOVE_LOOK_H

#include <Arduino.h>
#include <PB3DTimer.h>
#include "MoveBasic.h"
#include "MoveController.h"

class MoveLook{
public:
    MoveLook(MoveBasic* move_basic,
             MoveController* move_controller){
        _move_basic = move_basic;
        _move_controller = move_controller;
    }

    EMoveLookState look_around();
    void force_move();
    void reset();

    EMoveLookState get_look_state(){return _look_state;}
    uint8_t get_look_angle_index(){return _look_angle_index;}
    uint16_t get_look_total_time(){return _look_tot_time;}

private:
    MoveBasic* _move_basic = NULL;
    MoveController* _move_controller = NULL;

    Timer _look_timer = Timer();

    uint16_t _look_move_time = 2200;
    uint16_t _look_pause_time = 500;

    EMoveLookState _look_state = MOVE_LOOK_PAUSE;

    uint8_t _look_angle_index = 0;
    uint8_t _look_num_angs = 4;
    static const uint8_t _look_max_angs = 8;
    float _look_angles[_look_max_angs] = {30,0,-30,0,0,0,0,0};
    uint16_t _look_tot_time = _look_num_angs*(_look_move_time+_look_pause_time);
};
#endif
