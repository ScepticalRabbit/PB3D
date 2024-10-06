//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_PINS_H
#define PB3D_PINS_H

enum EMainPinout{
    ENCODER_PINA_LEFT = 2,
    ENCODER_PINB_LEFT = 3,
    ENCODER_PINA_RIGHT = 4,
    ENCODER_PINB_RIGHT = 5,
    MOOD_TASK_LED_PIN = 6,
    TAIL_SERVO = 8,
    SPEAKER_POUT = A0,
    I2C_SDA_PIN = A6,
    I2C_SCL_PIN = A7,
};

enum EGPIOPinout{
    // See lasers for first 10 pins7
    GPIO_PURR_VIBE = 12,
    GPIO_BUMPER_LEFT = 13,  // Pin: 5
    GPIO_BUMPER_RIGHT = 14, // Pin: 6
    GPIO_BUMPER_BACK = 15,  // Pin: 7
    GPIO_PIN_COUNT = 16
};

const uint8_t GPIO_PIN_MODES[16] = {OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    OUTPUT,
                                    INPUT_PULLUP,
                                    INPUT_PULLUP,
                                    INPUT_PULLUP};

#endif