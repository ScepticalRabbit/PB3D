//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#include <PB3DConstants.h>

#include "PB3DTimer.h"
#include "FilterMovAvg.h"
#include "FilterLowPass.h"


class Encoder{
public:
  Encoder(int8_t pinA, int8_t pinB);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  // Update for the right encoder '!='
  void update_not_equal();

  // Update for the left encoder '=='
  void update_equal();

  //-------------------------------------------------------------------------
  // SPEED CALCULATION FUNCTIONS
  void update_speed();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset_filter();
  void reset_filter(float inVal);

  int32_t get_count(){return _current_count;}
  void set_count(int32_t inCount){_current_count = inCount;}
  char get_direction(){return _direction;}
  double get_mm_per_count(){return _mm_per_count;}
  double get_raw_speed_mmps(){return _raw_speed_mmps;}
  double get_smooth_speed_mmps(){return _smooth_speed_mmps;}
  double get_raw_speed_cps(){return _raw_speed_cps;}
  double get_smooth_speed_cps(){return _smooth_speed_cps;}
  uint16_t get_speed_update_time(){return _speed_update_time;};

private:
  // CORE VARIABLES - Used by all types of encoder
  volatile int32_t _current_count;
  EEncoderDirection _direction = ENCODER_FORWARD;
  int8_t _pin_a = -1;
  int8_t _pin_b = -1;

  // SPEED VARIABLES - Used by robot wheel encoders
  Timer _speed_timer = Timer();

  // Characteristics of the motor/gears/wheel/encoder
  double _wheel_diam = 60.0;
  double _wheel_circ = _wheel_diam*PI;
  double _gear_ratio = 50.0;
  double _counts_on_encoder = 12.0;
  double _counts_per_rev = _gear_ratio*_counts_on_encoder;
  double _mm_per_count = _wheel_circ/_counts_per_rev;

  uint16_t _speed_update_time = 10;      // Default update time of 100Hz
  uint16_t _speed_filt_update_time = _speed_update_time-2;  // Must be less than speed update time.
  int32_t _count_prev_for_speed = 0;

  uint8_t _speed_filt_win = 10;
  float _speed_filt_alpha = 0.1; // NOTE: lower alpha for more smoothing!
  // NOTE: filter is called within speed update loop so set update time to
  // be less than the speed one so we update the filter at the same rate
  // as the speed
  //FilterMovAvg _speedFilt = FilterMovAvg(_speed_filt_win,_speed_update_time/2);
  FilterLowPass _speed_filt_mmps =
        FilterLowPass(_speed_filt_alpha,_speed_filt_update_time);
  double _raw_speed_mmps = 0.0;
  double _smooth_speed_mmps = 0.0;

  FilterLowPass _speed_filt_cps =
        FilterLowPass(_speed_filt_alpha,_speed_filt_update_time);
  double _raw_speed_cps = 0.0;
  double _smooth_speed_cps = 0.0;
};
#endif // ENCODER_H
