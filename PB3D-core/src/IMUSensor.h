//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include <Arduino.h>

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "PB3DTimer.h"

//#define IMU_DEBUG_TIMER
//#define IMU_DEBUG_RAW_OUTPUT
//#define IMU_DEBUG_ANGLES

class IMUSensor{
public:
  IMUSensor(){}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // GET, SET, RESET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}
  float get_roll_angle(){return _roll;}
  float get_pitch_angle(){return _pitch;}
  float get_head_angle(){return _heading;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  bool _init_IMU();

  bool _enabled = true;
  uint8_t _debug_count = 0;
  uint8_t _debug_freq = 10;

  const float _IMU_filter_freq = 100.0; // Hz
  const int16_t _IMU_update_time = int16_t(1000.0/_IMU_filter_freq); // ms
  Timer _IMU_timer = Timer();

  float _heading = 0.0;
  float _roll = 0.0;
  float _pitch = 0.0;

  Adafruit_FXOS8700 _fxos_accel_mag = Adafruit_FXOS8700(0x8700A, 0x8700B);
  Adafruit_FXAS21002C _fxas_gyro = Adafruit_FXAS21002C(0x0021002C);
  Adafruit_Sensor *_accel, *_gyro, *_mag;

  // pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion _filter; // slowest
  //Adafruit_Madgwick _filter;  // faster than NXP
  //Adafruit_Mahony _filter;  // fastest/smalleset

  // Need to change this for EEPROM but M0 and M4's use this
  Adafruit_Sensor_Calibration_SDFat _cal;

  // Event classes for holding sensor readings
  sensors_event_t _accel_event, _gyro_event, _mag_event;
};
#endif //
