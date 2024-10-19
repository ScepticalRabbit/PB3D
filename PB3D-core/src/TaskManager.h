//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "MoodManager.h"
#include "PB3DTimer.h"


class TaskManager{
public:
  TaskManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs);

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //----------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();
  void force_update();

  //----------------------------------------------------------------------------
  // Get, set and reset
  // Default setTask function uses default task durations
  void set_task(ETaskCode task);
  void set_task_duration(uint32_t task_dur);
  void assign_probability(EMoodCode mood);

  ETaskCode get_task(){return _task_code;}
  bool get_new_task_flag(){return _task_new_flag;}
  void set_new_task_flag(bool flag){_task_new_flag = flag;}
  void set_dance_duration(uint32_t duration){_dance_duration = duration;}
  void set_tantrum_duration(uint16_t duration){_tantrum_duration = duration;}
  void set_dance_update_flag(bool flag){_dance_update_flag = flag;}
  bool get_dance_update_flag(){return _dance_update_flag;}

  //----------------------------------------------------------------------------
  // TASK LED FUNCTIONS
  void task_LED_collision();
  void task_LED_explore();
  void task_LED_rest(uint8_t intensity);
  void task_LED_dance();
  void task_LED_tantrum();
  void task_LED_find_human();
  void task_LED_interact();
  void task_LED_pickedup_Ok();
  void task_LED_pickedup_panic();
  void task_LED_pause(uint16_t pauseTime);
  void task_LED_find_light();
  void task_LED_find_dark();
  void task_LED_find_sound();
  void task_LED_test();
  void task_LED_off();

  //------------------------------------------------------------------------
  // HSV LEDS
  void task_LED_hue(uint16_t hue);
  void task_LED_HSV(uint16_t hue, uint8_t sat, uint8_t value);
  void task_LED_colour(uint16_t col);
  void task_LED_colour(uint16_t colL, uint16_t colR);
  void task_LED_CSV(uint16_t col, uint8_t sat, uint8_t val);
  void task_LED_CSV(uint16_t colL,uint16_t colR,
                    uint8_t satL,uint8_t satR,
                    uint8_t valL, uint8_t valR);

private:
  void _update();
  void _set_task_probability(const int8_t probabilities[]);
  uint8_t _calc_falling_LED_val(uint16_t time_int);
  uint8_t _calc_rising_LED_val(uint16_t time_int);

  //------------------------------------------------------------------------
  // TASK Variables
  // [1.explore,2.rest,3.dance,4.findhuman,5.findsound,6.findlight,7.finddark]
  ETaskCode _task_code = TASK_EXPLORE;
  int8_t _task_percent = 0;

  const static int8_t _task_count = 7;
  int8_t _task_prob_bounds[_task_count] = {30,40,55,70,85,95,100};

  const int8_t _task_prob_test[_task_count] =    {100,0,0,0,0,0,0};
  const int8_t _task_prob_neutral[_task_count] = {30,10,15,15,15,10,5};
  const int8_t _task_prob_happy[_task_count] =   {25,5,20,20,15,15,0};
  const int8_t _task_prob_sad[_task_count] =     {40,20,5,5,10,0,20};
  const int8_t _task_prob_angry[_task_count] =   {40,0,15,15,15,10,5};
  const int8_t _task_prob_scared[_task_count] =  {30,5,10,10,10,0,35};

  uint32_t _task_duration = 7000;
  const uint32_t _task_duration_min = 5000;
  const uint32_t _task_duration_max = 15000;
  bool _task_new_flag = true;

  bool _dance_update_flag = false;
  uint32_t _dance_duration = 0;
  uint16_t _tantrum_duration = 0;

  uint16_t _huePounce = (65536 * 1)/12;

  Timer _taskTimer = Timer();

  Timer _LED_timer = Timer();
  bool _LED_switch = true;
  const uint16_t _LED_on_off_time = 500;
  const uint16_t _LED_slope_time = 1000;

  Adafruit_NeoPixel_ZeroDMA* _task_LEDs;
};
#endif // TASK_H
