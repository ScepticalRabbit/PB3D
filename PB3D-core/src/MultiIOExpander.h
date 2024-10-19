//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MULTIIOEXPANDER_H
#define MULTIIOEXPANDER_H

#include <Arduino.h>
#include <Adafruit_PCF8574.h>

#include "PB3DI2CAddresses.h"

class MultiIOExpander{
public:
    MultiIOExpander(const uint8_t pin_modes[]);

    bool begin();

    bool digital_write(uint8_t pin, bool val);
    bool digital_read(uint8_t pin);

    bool enabled = true;

    static const uint8_t num_expanders = 2;
    const uint8_t pins_per_expander = 8;
    static const uint8_t total_pins = 16;

private:
    uint8_t _pin_modes[total_pins];
    Adafruit_PCF8574 _gpio_expander_1 = Adafruit_PCF8574();
    Adafruit_PCF8574 _gpio_expander_2 = Adafruit_PCF8574();
    Adafruit_PCF8574* _gpio_expanders[num_expanders] = {&_gpio_expander_1,
                                                        &_gpio_expander_2};
};

#endif