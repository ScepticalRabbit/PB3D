#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Speaker.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "Speaker.h"


//----------------------------------------------------------------------------
// BEGIN: called once during SETUP
void Speaker::begin(){
    // NOTE: see adafruit guidance about metro M4 dac here:
    // https://learn.adafruit.com/adafruit-metro-m4-express-featuring-atsamd51/adapting-sketches-to-m0
    // pinMode(SPEAKER_POUT,OUTPUT);

    digitalWrite(SPEAKER_POUT,LOW);
    _sound_timer.start(0);
    _note_timer.start(0);
    _song_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void Speaker::update(){
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
// SET, GET, RESET
void Speaker::set_sound_codes(uint8_t codes[], uint8_t num_codes){
    for(uint8_t ii=0; ii<num_codes; ii++){
        _sound_codes[ii] = codes[ii];

        if(ii >= _num_codes){
            return;
        }
    }
}

void Speaker::set_sound_freqs(uint16_t freqs[], uint8_t num_freqs){
    for(uint8_t ii=0; ii<num_freqs; ii++){
        _sound_freqs[ii] = freqs[ii];

        if(ii >= _num_freqs){
            return;
        }
    }
}

void Speaker::set_sound_durations(uint16_t durations[], uint8_t num_durations){
    for(uint8_t ii=0; ii<num_durations; ii++){
        _sound_durations[ii] = durations[ii];

        if(ii >= _num_durs){
            return;
        }
    }
}

uint32_t Speaker::get_total_sound_duration(){
    uint32_t duration_sum = 0;
    for(uint8_t ii=0; ii<_num_durs; ii++){
        duration_sum += _sound_durations[ii];
    }
    return duration_sum;
}

void Speaker::off(){
    noTone(SPEAKER_POUT);
    digitalWrite(SPEAKER_POUT,LOW);
}


//----------------------------------------------------------------------------
// MAIN SOUND - ON
void Speaker::make_sound(uint8_t code,
                         uint16_t freq1, uint16_t freq2,
                         uint16_t on_dur, uint16_t off_dur){
    if(_sound_start){
        _sound_start = false;
        _sound_count = SOUND_PLAY;
        _note_timer.start(on_dur);
    }

    if(_note_timer.finished()){
        _sound_count++;
        if(_sound_count == SOUND_PAUSE){
        _note_timer.start(off_dur);
        }
        else if(_sound_count >= SOUND_END){
        _sound_count = SOUND_END;
        }
    }

    if(_sound_count == SOUND_PLAY){

        switch(code){
            case SPEAKER_BEEP:
                beep(freq1);
                break;
            case SPEAKER_SLIDE:
                slide(freq1,freq2,on_dur);
                break;
            case SPEAKER_GROWL:
                growl();
                break;
            case SPEAKER_SCREECH:
                screech(freq1,freq2);
                break;
            case SPEAKER_SNORE:
                snore();
                break;
            default:
                off();
                break;
        }
    }
}

//--------------------------------------------------------------------------------------------
// MAIN SOUND - RESET
void Speaker::reset(){
    _sound_start = true;
    _sound_count = SOUND_PLAY;
    _sound_code_index = 0;
}

//--------------------------------------------------------------------------------------------
// SUB-SOUND - BEEP
void Speaker::beep(uint16_t beepFreq){
    tone(SPEAKER_POUT,beepFreq);
}

//--------------------------------------------------------------------------------------------
// SUB-SOUND - SLIDE
void Speaker::slide(uint16_t start_freq, uint16_t end_freq, uint16_t on_dur){
    uint16_t curr_freq = _calc_slide_freq(start_freq,
                                        end_freq,
                                        on_dur,
                                        _note_timer.get_time());
    tone(SPEAKER_POUT,curr_freq);
}

//--------------------------------------------------------------------------------------------
// SUB-SOUND - GROWL
void Speaker::growl(){
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
void Speaker::screech(uint16_t freq1, uint16_t freq2){
    if(_rand_timer.finished()){
        _rand_timer.start(_screech_rand_interval);
        uint16_t curr_freq = _gen_random_freq(freq1,freq2);
        tone(SPEAKER_POUT,curr_freq);
    }
}

//----------------------------------------------------------------------------
// SUB-SOUND - SNORE
void Speaker::snore(){
    _snore_curr_freq = round(float(_snore_start_freq) +
                        _snore_slope*float(_note_timer.get_time()));
    tone(SPEAKER_POUT,_snore_curr_freq);
}



