//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef COLLISIONESCAPER_H
#define COLLISIONESCAPER_H

#include <Arduino.h>
#include "MoveManager.h"
#include "CollisionDangerCodes.h"

#define ESCAPE_NOREV 1
#define ESCAPE_REV 0

class CollisionEscaper{
    public:
        CollisionEscaper(){};

        void set_move_obj(MoveManager* inMove);
        void update_escape_decision(uint8_t check_vec[]);
        void set_escape_start(uint8_t check_vec[]);
        void escape();
        bool get_escape_flag();
        int8_t get_escape_turn(uint8_t check_vec[]);

        uint8_t get_escape_count(){return _escape_count;}
        float get_escape_dist(){return _escape_dist;}
        float get_escape_angle(){return _escape_angle;}

    private:
        float _get_rand_turn_dir();

        MoveManager* _move_manager = NULL;
        uint8_t _escape_count = 3;
        float _escape_angle = 45.0;
        float _escape_dist = 180.0;
        const static uint8_t _escape_num_steps = 3;
        float _def_mod_turn = 45.0;
        float _def_hard_turn = 90.0;
        float _def_rev_dist = -180.0;
};

#endif