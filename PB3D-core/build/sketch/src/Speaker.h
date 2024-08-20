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
#include "PB3DConstants.h"
#include "Timer.h"

#define SPEAKER_POUT A0


class Speaker{
public:
  Speaker(){}

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    // NOTE: see adafruit guidance about metro M4 dac here:
    // https://learn.adafruit.com/adafruit-metro-m4-express-featuring-atsamd51/adapting-sketches-to-m0
    // pinMode(SPEAKER_POUT,OUTPUT);

    digitalWrite(SPEAKER_POUT,LOW);
    _sound_timer.start(0);
    _note_timer.start(0);
    _song_timer.start(0);
  }

  void off(){
    noTone(SPEAKER_POUT);
    digitalWrite(SPEAKER_POUT,LOW);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    // If the current sound has ended increment to play the next one
    if(_sound_count == SOUND_END){
      _sound_start = true;
      _sound_count = SOUND_PLAY;
      _sound_code_index++;

      // If we have run out of codes in the array wait until reset is called
      // Reset must be called externally to this class
      if(_sound_code_index >= _num_codes){
        _sound_count = SOUND_END;
        _sound_code_index = _num_codes-1;
      }
    }

    if(_sound_timer.finished()){
      _sound_timer.start(_sound_update_time);

      uint8_t ss = _sound_code_index;
      uint8_t ff = 2*_sound_code_index;

      _curr_sound_code = _sound_codes[ss];
      if(_curr_sound_code == SPEAKER_SNORE){
        _sound_durations[ff] = _snore_duration;
      }
      else if (_curr_sound_code == SPEAKER_GROWL){
        _sound_durations[ff] = _growl_duration;
      }

      make_sound(_curr_sound_code,
                _sound_freqs[ff],_sound_freqs[ff+1],
                _sound_durations[ff] ,_sound_durations[ff+1]);
    }
  }

  //----------------------------------------------------------------------------
  // SETTERS
  void set_sound_codes(uint8_t inCodes[], uint8_t numCodes){
    for(uint8_t ii=0; ii<numCodes; ii++){
      _sound_codes[ii] = inCodes[ii];
    }
  }

  void set_sound_freqs(uint16_t inFreqs[], uint8_t numFreqs){
    for(uint8_t ii=0; ii<numFreqs; ii++){
      _sound_freqs[ii] = inFreqs[ii];
    }
  }

  void set_sound_durations(uint16_t inDurs[], uint8_t numDurs){
    for(uint8_t ii=0; ii<numDurs; ii++){
      _sound_durations[ii] = inDurs[ii];
    }
  }

  //----------------------------------------------------------------------------
  // GETTERS
  uint8_t get_curr_sound_code(){return _curr_sound_code;}
  uint8_t get_sound_count(){return _sound_count;}
  uint8_t get_num_codes(){return _num_codes;}
  uint8_t get_num_freqs(){return _num_freqs;}
  uint8_t get_num_durs(){return _num_durs;}

  uint32_t get_total_sound_duration(){
    uint32_t durSum = 0;
    for(uint8_t ii=0; ii<_num_durs; ii++){
      durSum += _sound_durations[ii];
    }
    return durSum;
  }

  //----------------------------------------------------------------------------
  // MAIN SOUND - ON
  void make_sound(uint8_t inCode, uint16_t freq1, uint16_t freq2, uint16_t onDur, uint16_t offDur){
    if(_sound_start){
      _sound_start = false;
      _sound_count = SOUND_PLAY;
      _note_timer.start(onDur);
    }

    if(_note_timer.finished()){
      _sound_count++;
      if(_sound_count == SOUND_PAUSE){
        _note_timer.start(offDur);
      }
      else if(_sound_count >= SOUND_END){
        _sound_count = SOUND_END;
      }
    }

    if(_sound_count == SOUND_PLAY){
      if(inCode == SPEAKER_BEEP){
        beep(freq1);
      }
      else if(inCode == SPEAKER_SLIDE){
        slide(freq1,freq2,onDur);
      }
      else if(inCode == SPEAKER_GROWL){
        growl();
      }
      else if(inCode == SPEAKER_SCREECH){
        screech(freq1,freq2);
      }
      else if(inCode == SPEAKER_SNORE){
        snore();
      }
      else{
        off();
      }
    }
    else{
      off();
    }
  }

  //--------------------------------------------------------------------------------------------
  // MAIN SOUND - RESET
   void reset(){
    //Serial.println(F("SPEAKER: ext reset."));
    _sound_start = true;
    _sound_count = SOUND_PLAY;
    _sound_code_index = 0;
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - BEEP
  void beep(uint16_t beepFreq){
      tone(SPEAKER_POUT,beepFreq);
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - SLIDE
   void slide(uint16_t startFreq, uint16_t endFreq, uint16_t onDur){
    uint16_t currFreq = _calc_slide_freq(startFreq,endFreq,onDur,_note_timer.get_time());
    tone(SPEAKER_POUT,currFreq);
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - GROWL
   void growl(){
    _growl_oscillator = !_growl_oscillator;
    if(_growl_oscillator){
      tone(SPEAKER_POUT,_growl_freq1);
    }
    else{
      tone(SPEAKER_POUT,_growl_freq2);
    }
  }

  //----------------------------------------------------------------------------
  // SUB-SOUND - RANDOM SCREECH
  void screech(uint16_t freq1, uint16_t freq2){
    if(_rand_timer.finished()){
      _rand_timer.start(_screech_rand_interval);
      uint16_t currFreq = _gen_random_freq(freq1,freq2);
      tone(SPEAKER_POUT,currFreq);
    }
  }

  //----------------------------------------------------------------------------
  // SUB-SOUND - SNORE
   void snore(){
    _snore_curr_freq = round(float(_snore_start_freq) + _snore_slope*float(_note_timer.get_time()));
    tone(SPEAKER_POUT,_snore_curr_freq);
  }

private:
  uint16_t _calc_slide_freq(uint16_t startNote, uint16_t endNote, uint16_t dur, uint16_t currTime){
    float slideSlope = (float(startNote)-float(endNote))/float(0.0-dur);
    uint16_t currFreq = round(float(startNote) + slideSlope*float(currTime));
    return currFreq;
  }

  uint16_t _gen_random_freq(uint16_t freq1, uint16_t freq2){
    uint16_t randFreq = 0;
    if(freq1 > freq2){
      randFreq = random(freq2,freq1);
    }
    else if(freq2 > freq1){
      randFreq = random(freq1,freq2);
    }
    return randFreq;
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
  uint8_t _sound_codes[_num_codes] = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
  uint16_t _sound_freqs[_num_freqs] = {NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5};
  uint16_t _sound_durations[_num_durs] = {100,100,100,100,100,100,100,100};

  // TIMERS
  Timer _sound_timer = Timer();
  Timer _song_timer = Timer();
  Timer _note_timer = Timer();

  // BEEP BEEP Variables
  /*
  bool _bbStart = true;
  uint16_t _bbNoteDur = 100;
  uint16_t _bbPause = 100;
  uint8_t _bbNumBB = 2;
  uint8_t _bbNoteCount = 0;
  uint8_t _bbCount = 0;
  */

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
