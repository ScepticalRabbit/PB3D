//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef COLLISIONSTRATEGY_H
#define COLLISIONSTRATEGY_H

#include <Arduino.h>

#include "PB3DConstants.h"


class CollisionAvoidBasic{
    public:
        CollisionAvoidBasic(int16_t close, int16_t far){
            _close = close;
            _far = far;
        }

        EDangerCode get_collision_code(int16_t range);

    private:
        int16_t _close = 0;
        int16_t _far = 0;
};


class CollisionAvoidSlow{
    public:
        CollisionAvoidSlow(int16_t close, int16_t far, int16_t slow){
            _close = close;
            _far = far;
            _slow = slow;
        }

        EDangerCode get_collision_code(int16_t range);

    private:
        int16_t _close = 0;
        int16_t _far = 0;
        int16_t _slow = 0;
};


class CliffAvoid{
    public:
        CliffAvoid(int16_t close, int16_t far){
            _close = close;
            _far = far;
        }

        EDangerCode get_collision_code(int16_t range);

    private:
        int16_t _close = 0;
        int16_t _far = 0;
};

class CollisionCliffAvoid{
    public:
        CollisionCliffAvoid(int16_t col_close,
                            int16_t col_far,
                            int16_t cliff_close,
                            int16_t cliff_far){
            _col_close = col_close;
            _col_far = col_far;
            _cliff_close = cliff_close;
            _cliff_far = cliff_far;
        }

        EDangerCode get_collision_code(int16_t range);

    private:
        int16_t _col_close = 0;
        int16_t _col_far = 0;
        int16_t _cliff_close = 0;
        int16_t _cliff_far = 0;
};




#endif