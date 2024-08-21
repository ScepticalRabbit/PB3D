#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/PB3DPins.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_PINS
#define PB3D_PINS

enum EPinout{
    ENCODER_PINA_LEFT = 2,
    ENCODER_PINB_LEFT = 3,
    ENCODER_PINA_RIGHT = 4,
    ENCODER_PINB_RIGHT = 5,
    MOOD_LED_PIN = 6,
    TAIL_SERVO = 8,
    SPEAKER_POUT = A0,
    I2C_SDA_PIN = A6,
    I2C_SCL_PIN = A7,
};

#endif