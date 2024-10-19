//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef WHEEL_DATA_H
#define WHEEL_DATA_H

#include <Arduino.h>

class WheelData{
public:
    //WheelData(){};
    // Wheel base constant parameters
    // NOTE: D(inner) = 122mm, D(outer) = 160mm, D(avg) = 141mm
    const float base = 172.0; // UPDATED: 1st Jan 2023 - new stable geom chassis with large wheels
    const float circumference = base*PI;
    const float circ_per_degree = (base*PI)/(360.0);
    // FIXED factor of 2 by adding encoder interrupt
};

#endif
