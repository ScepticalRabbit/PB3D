#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoodManager.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOODMANAGER_H
#define MOODMANAGER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

#include "PB3DConstants.h"
#include "Timer.h"

// Mood LEDs
#define MOOD_PIN 6
#define MOOD_NUMPIX 4

class MoodManager{
public:
  MoodManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){_moodLEDs = RGBLEDs;}

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //----------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //----------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool enabled){_enabled = enabled;}
  void reset_mood(){_set_mood(_mood_code);}
  EMoodCode get_mood(){return _mood_code;}
  int8_t get_power_diff(){return _mood_power_diff_array[_mood_code];}
  float get_speed_fact(){return _mood_speed_fact_array[_mood_code];}
  bool get_new_mood_flag(){return _mood_new_flag;}
  void set_new_mood_flag(bool enabled){_mood_new_flag = enabled;}

  void set_mood(EMoodCode mood);

  //----------------------------------------------------------------------------
  // Mood Score Functions
  int8_t get_mood_score(){return _mood_score;}
  void set_mood_score(int8_t score){
    _mood_score = constrain(score,_mood_score_min,_mood_score_max);
  }
  void reset_mood_score(){_mood_score = 0;}

  void mod_mood_score(int8_t modifier);
  void inc_mood_score();
  void dec_mood_score();

  //---------------------------------------------------------------------------
  // Mood LED Functions
  void mood_LED_neutral();
  void mood_LED_happy();
  void mood_LED_angry();
  void mood_LED_sad();
  void mood_LED_scared();
  void mood_LED_curious();
  void mood_LED_test();
  void mood_LED_off();

private:
  void _set_mood(EMoodCode mood);

  // Moods: [neutral,happy,sad,angry,scared]
  // Mood changes task probabilities
  bool _enabled = true;

  EMoodCode _mood_code = MOOD_NEUTRAL;
  int16_t _mood_percent = 0;
  int16_t _mood_neg_percent = 0;
  bool  _mood_new_flag = true;

  // Mood score variables
  int8_t _mood_score = 0;
  int8_t _mood_score_increment = 10;
  const int8_t _mood_score_min = -50;
  const int8_t _mood_score_max = 50;

  Timer _mood_timer = Timer();
  uint32_t _mood_duration = 12000;
  const uint32_t _mood_min_duration = 10000;
  const uint32_t _mood_max_duration = 40000;

  // Moods: [neutral,happy,sad,angry,scared]
  // Mood axis = [negative,neutral,positive]
  // Negative moods = [sad,angry,scared]
  const int8_t _mood_prob_array[5] =   {40,70,80,90,100};
  const int8_t _mood_prob_axis_array[3] = {25,75,100};
  const int8_t _mood_prob_neg_array[3] = {33,66,100};
  const int8_t _mood_power_diff_array[5] =  {0,10,-30,20,-10};
  const float _mood_speed_fact_array[5] = {1.0,1.1,0.7,1.2,0.9};

  // Pointer to the mood expression LEDs
  Adafruit_NeoPixel_ZeroDMA* _moodLEDs = NULL;
};
#endif // MOOD_H
