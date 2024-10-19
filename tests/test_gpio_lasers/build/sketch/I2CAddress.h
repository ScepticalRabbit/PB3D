#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/I2CAddress.h"
//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_I2CADDR_H
#define PB3D_I2CADDR_H

#define ADDR_GPIO 0x21

// NOTE: The default address for the VL53L0X is 0x29
#define ADDR_LSR_CC 0x30
#define ADDR_LSR_UC 0x31
#define ADDR_LSR_DL 0x32
#define ADDR_LSR_DR 0x33
#define ADDR_LSR_HL 0x34
#define ADDR_LSR_HR 0x35
#define ADDR_LSR_LL 0x36
#define ADDR_LSR_RR 0x37
#define ADDR_LSR_BB 0x38
#define ADDR_LSR_AA 0x39

#endif
