//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef COLLISIONESCAPER_H
#define COLLISIONESCAPER_H

#include <Arduino.h>

#include <PB3DConstants.h>
#include "MoveManager.h"


class CollisionEscaper{
    public:
        CollisionEscaper(){};

        void set_move_obj(MoveManager* inMove);
        void update_escape_decision(uint8_t check_bumpers[],
                                    uint8_t check_lasers[]);
        //void set_escape_start(uint8_t check_vec[]);
        void escape();
        bool get_escape_state();
        EMoveBasic get_escape_turn();

        uint8_t get_escape_count(){return _escape_state;}
        float get_escape_dist(){return _escape_dist;}
        float get_escape_angle(){return _escape_angle;}

    private:
        float _get_rand_turn_dir();

        MoveManager* _move_manager = NULL;
        EEscapeState _escape_state = ESC_STATE_COMPLETE;
        float _escape_angle = 45.0;
        float _escape_dist = 180.0;
        const static uint8_t _escape_num_steps = 3;

        // TODO: check that negative is turn right!
        const float _mod_turn_left = 45.0;
        const float _mod_turn_right = -45.0;
        const float _hard_turn_left = 90.0;
        const float _hard_turn_right = -90.0;
        const float _def_rev_dist = -180.0;
};

#endif