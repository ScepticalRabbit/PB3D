#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Speaker.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>

#include <PB3DPins.h>
#include <PB3DConstants.h>
#include "Timer.h"


class Speaker{
public:
  Speaker(){}

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //----------------------------------------------------------------------------
  // SET, GET, RESET
  void set_sound_codes(uint8_t codes[], uint8_t num_codes);
  void set_sound_freqs(uint16_t freqs[], uint8_t num_freqs);
  void set_sound_durations(uint16_t durations[], uint8_t num_durations);
  uint32_t get_total_sound_duration();
  void off();

  uint8_t get_curr_sound_code(){return _curr_sound_code;}
  uint8_t get_sound_count(){return _sound_count;}
  uint8_t get_num_codes(){return _num_codes;}
  uint8_t get_num_freqs(){return _num_freqs;}
  uint8_t get_num_durs(){return _num_durs;}

  //----------------------------------------------------------------------------
  // MAIN SOUND - ON
  void make_sound(uint8_t code,
                  uint16_t freq1, uint16_t freq2,
                  uint16_t on_dur, uint16_t off_dur);
  void reset();

  //----------------------------------------------------------------------------
  // SUB-SOUNDS
  void beep(uint16_t beepFreq);
  void slide(uint16_t start_freq, uint16_t end_freq, uint16_t on_dur);
  void growl();
  void screech(uint16_t freq1, uint16_t freq2);
  void snore();

private:
  uint16_t _calc_slide_freq(uint16_t start_note,
                            uint16_t end_note,
                            uint16_t dur,
                            uint16_t curr_time){
    float slide_slope = (float(start_note)-float(end_note)) /
                        float(0.0-dur);
    uint16_t curr_freq = round(float(start_note) +
                         slide_slope*float(curr_time));
    return curr_freq;
  }

  uint16_t _gen_random_freq(uint16_t freq1, uint16_t freq2){
    uint16_t rand_freq = 0;
    if(freq1 > freq2){
      rand_freq = random(freq2,freq1);
    }
    else if(freq2 > freq1){
      rand_freq = random(freq1,freq2);
    }
    return rand_freq;
  }


  uint8_t _curr_sound_code = SPEAKER_OFF;
  bool _sound_start = false;
  uint16_t _sound_update_time = 20;

  // FREQUENCIES AND TIMES
  uint8_t _sound_code_index = 0;
  uint8_t _sound_count = 0;

  const static uint8_t _num_codes = 4;
  const static uint8_t _num_freqs = 8;
  const static uint8_t _num_durs = 8;
  uint8_t _sound_codes[_num_codes] = {SPEAKER_OFF,
                                      SPEAKER_OFF,
                                      SPEAKER_OFF,
                                      SPEAKER_OFF};
  uint16_t _sound_freqs[_num_freqs] = {NOTE_C5,
                                       NOTE_C5,
                                       NOTE_C5,
                                       NOTE_C5,
                                       NOTE_C5,
                                       NOTE_C5,
                                       NOTE_C5};
  uint16_t _sound_durations[_num_durs] = {100,100,100,100,100,100,100,100};

  // TIMERS
  Timer _sound_timer = Timer();
  Timer _song_timer = Timer();
  Timer _note_timer = Timer();

  // SLIDE Variables
  bool _startSlide = true;
  uint8_t _slideRepCount = 0;
  uint8_t _slideNoteCount = 0;

  // GROWL Variables
  bool _startGrowl = true;
  bool _growl_oscillator = true;
  uint16_t _growl_duration = 1000;
  uint16_t _growl_freq1 = NOTE_B2;
  uint16_t _growl_freq2 = NOTE_B3;

  // SCREECH Variables
  bool _start_screech = true;
  uint16_t _screech_rand_interval = 25;
  Timer _rand_timer = Timer();

  // SNORE Variables
  bool _start_snore = true;
  uint16_t _snore_duration = 2000;
  uint16_t _snore_start_freq = NOTE_C4;
  uint16_t _snoreEndFreq = NOTE_B2;
  float _snore_slope = -float(_snore_start_freq -_snoreEndFreq)/float(_snore_duration);
  uint16_t _snore_curr_freq = _snore_start_freq;
};
#endif // SPEAKER_H
