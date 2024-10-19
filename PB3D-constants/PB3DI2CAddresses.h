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


#define ADDR_FOLLOW_XIAO_1 0x09 // Sound / DigIO for setup
#define ADDR_FOLLOW_XIAO_2 0x11 // Radio TX / Active DigIO

#define ADDR_TOUCHSENS 0x08

#define ADDR_GPIO_1 0x20 // first 8 lasers
#define ADDR_GPIO_2 0x21 // last two lasers + 3 bumpers (5(L),6(R),7(B))

// NOTE: The default address for the VL53L0X is 0x29
#define ADDR_LSR_CC 0x30 // Multi-IO Pin: 0
#define ADDR_LSR_UC 0x31 // Multi-IO Pin: 1
#define ADDR_LSR_DL 0x32 // Multi-IO Pin: 2
#define ADDR_LSR_DR 0x33 // Multi-IO Pin: 3
#define ADDR_LSR_HL 0x34 // Multi-IO Pin: 4
#define ADDR_LSR_HR 0x35 // Multi-IO Pin: 5
#define ADDR_LSR_LL 0x36 // Multi-IO Pin: 6
#define ADDR_LSR_RR 0x37 // Multi-IO Pin: 7
#define ADDR_LSR_BB 0x38 // Multi-IO Pin: 0
#define ADDR_LSR_AA 0x39 // Multi-IO Pin: 1

#define ADDR_MOTOR_SHIELD 0x60

#define ADDR_IR_PRES_SENS 0x64

#define ADDR_TCA_I2CMULTIPLEX 0x70


#endif
