//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include <PB3DConstants.h>
#include <PB3DI2CAddresses.h>

// Debug Flags
//#define DEBUG_STATE
//#define DEBUG_SOUND_TIME
//#define DEBUG_ENVSAMP

//------------------------------------------------------------------------------
// SENSOR SELECT
//#define EAR_GROVE
#define EAR_DFROB

#ifdef EAR_GROVE
  // 12bit = 1585 average with Grove mems mic
  int16_t env_avg_std = 1585;

  enum EEarAPins{
    EAR_L_AIN = A7,
    EAR_R_AIN = A9,
  };
#endif

#ifdef EAR_DFROB
  // 12bit = 1845 average with DFRobot Fermion Mic
  // 10bit = 460 average with DFRobot Fermion Mic
  int16_t env_avg_std = 1845;

  enum EEarAPins{
    EAR_L_AIN = A8,
    EAR_R_AIN = A9,
  };
#endif


//---------------------------------------------------------------------------
// SOUND LOC/EAR VARS
// Codes for ear state
enum EEarMode{
    EAR_MODE_ENVSAMP = 0,
    EAR_MODE_SNDLOC,
};
EEarMode ear_mode = EAR_MODE_ENVSAMP;

// Variables for sound location code
// 0: no sound
// 1: forward -> sound but uncertain direction
// 2: left
// 3: right
byte ear_state = 0; // byte to send

byte rec_byte = B00000000;

uint16_t sample_interval = 10;
uint32_t sample_last_time = 0;
uint16_t num_env_samples = 100;
uint32_t env_sum_left = 0;
uint32_t env_sum_right = 0;
int16_t env_sample_ind = 0;
int16_t env_avg_left = env_avg_std;
int16_t env_avg_right = env_avg_std;
int16_t env_diff_sum_left = 0;
int16_t env_diff_sum_right = 0;
int16_t env_SD_left = 0;
int16_t env_SD_right = 0;

bool found_sound_both = false;
int32_t sound_LR_timer_diff = 0;

bool found_sound_left = false;
bool found_sound_right = false;
uint32_t sound_time_stamp_left = 0;
uint32_t sound_time_stamp_right = 0;
int16_t sound_left = 0;
int16_t sound_right = 0;
int16_t sound_offset_left = 500;
int16_t sound_offset_right = 500;

uint32_t ear_reset_time = 500;
uint32_t ear_prev_reset_time = 0;

int16_t sound_upper_left = env_avg_left + sound_offset_left;
int16_t sound_upper_right = env_avg_right + sound_offset_right;
int16_t sound_lower_left = env_avg_left - sound_offset_left;
int16_t sound_lower_right = env_avg_right - sound_offset_right;
int16_t time_diff_min_mus = 75;
int16_t time_diff_max_mus = 400;

uint32_t time_start = 0;
uint32_t time_end = 0;
uint32_t time_elapsed = 0;
uint16_t num_samples = 1000;
int16_t sample_val = 0;

byte prev_byte = B00000000;

//---------------------------------------------------------------------------
void setup(){
  Wire.begin(ADDR_FOLLOW_XIAO_1);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(115200);

  // Set the analog read resolution for SAMD51
  analogReadResolution(12);
}


void loop(){
  //----------------------------------------------------------------------------
  // SOUND LOCATION

  // HACK: for some reason the rec_byte keeps being set to 255 (B11111111)
  // So we ignore this case here to prevent errors
  if((rec_byte != B11111111) && ((rec_byte & B01000000) == B01000000)){
    ear_mode = EAR_MODE_ENVSAMP;
  }

  if(ear_mode == EAR_MODE_SNDLOC){
    locate_sound();
  }
  else{
    sample_environment();
  }

  //tttttttttttttttttttttttttttttttttttttttttttttttt
  // TIMING CODE
  #ifdef DEBUG_SOUND_TIME
  time_start = micros();
  for(int ii=0; ii<num_samples; ii++) {
    found_sound_left = true;
    locate_sound();
  }
  time_end = micros();
  time_elapsed = time_end - time_start;
  float tAvgMicroS = float(time_elapsed)/float(num_samples);
  float tAvgS = tAvgMicroS/1000000.0;
  float freqAvg = 1.0/tAvgS;

  Serial.print("Avg. Time per Sample: ");
  Serial.print(tAvgMicroS);
  Serial.println(" us");
  Serial.print("Avg. Frequency: ");
  Serial.print(freqAvg);
  Serial.println(" Hz");
  Serial.println();
  delay(5000);
  #endif

  //tttttttttttttttttttttttttttttttttttttttttttttttt

}

//---------------------------------------------------------------------------
// FUNCTIONS - I2C COMMS
void receiveEvent(int numBytes) {
  rec_byte = Wire.read();
}

void requestEvent() {
  Wire.write(ear_state);
}

//---------------------------------------------------------------------------
// FUNCTIONS - SOUND LOCATION
void sample_environment(){
  ear_state = EAR_COM_SENV;

  if((millis()-sample_last_time)>sample_interval){
    sample_last_time = millis();
    int16_t LSamp = analogRead(EAR_L_AIN);
    int16_t RSamp = analogRead(EAR_R_AIN);
    //int16_t RSamp = 0;

    // Sum of values to calculate average
    env_sum_left = env_sum_left + LSamp;
    env_sum_right = env_sum_right + RSamp;

    // Difference to prelim average for SD calc
    env_diff_sum_left = env_diff_sum_left + abs(env_avg_left - LSamp);
    env_diff_sum_right = env_diff_sum_right + abs(env_avg_right - RSamp);

    env_sample_ind++;
  }

  if(env_sample_ind >= num_env_samples){
    env_sample_ind = 0;
    ear_mode = EAR_MODE_SNDLOC;
    ear_state = EAR_COM_NOSOUND;

    env_avg_left = int(float(env_sum_left)/float(num_env_samples));
    env_avg_right = int(float(env_sum_right)/float(num_env_samples));
    env_SD_left = int(float(env_diff_sum_left)/float(num_env_samples));
    env_SD_right = int(float(env_diff_sum_right)/float(num_env_samples));

    if(6*env_SD_left >= sound_offset_left){sound_offset_left = 6*env_SD_left;}
    if(6*env_SD_right >= sound_offset_right){sound_offset_right = 6*env_SD_right;}

    sound_upper_left = env_avg_left + sound_offset_left;
    sound_upper_right = env_avg_right + sound_offset_right;
    sound_lower_left = env_avg_left - sound_offset_left;
    sound_lower_right = env_avg_right - sound_offset_right;

    #ifdef DEBUG_ENVSAMP
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.println("EAR-ENVSAMP");
    Serial.print("L Sum: ");
    Serial.print(env_sum_left);
    Serial.print(" , ");
    Serial.print("R Sum: ");
    Serial.println(env_sum_right);
    Serial.print("L Avg: ");
    Serial.print(env_avg_left);
    Serial.print(" , ");
    Serial.print("R Avg: ");
    Serial.println(env_avg_right);
    Serial.print("L SD: ");
    Serial.print(env_SD_left);
    Serial.print(" , ");
    Serial.print("R SD: ");
    Serial.println(env_SD_right);
    Serial.print("L 6xSD: ");
    Serial.print(6*env_SD_left);
    Serial.print(" , ");
    Serial.print("R 6xSD: ");
    Serial.println(6*env_SD_right);
    #endif

    // Reset environment calc sums
    env_sum_left = 0, env_sum_right = 0;
    env_diff_sum_left = 0, env_diff_sum_right = 0;
  }
}

void locate_sound(){
  if(!found_sound_left){
    sound_left = analogRead(EAR_L_AIN);
    if(((sound_left > sound_upper_left) || (sound_left < sound_lower_left))){
      sound_time_stamp_left = micros();
      found_sound_left = true;
    }
  }

  if(!found_sound_right){
    sound_right = analogRead(EAR_R_AIN);
    if(!found_sound_right && ((sound_right > sound_upper_right) || (sound_right < sound_lower_right))){
      sound_time_stamp_right = micros();
      found_sound_right = true;
    }
  }

  if(found_sound_left && found_sound_right){
    found_sound_both = true;
    sound_LR_timer_diff = sound_time_stamp_left - sound_time_stamp_right;

    if(abs(sound_LR_timer_diff) > time_diff_max_mus){
      // if greater than max time then this is a reflection
      ear_state = EAR_COM_FORWARD;
    }
    else if(sound_LR_timer_diff >= time_diff_min_mus){
      // if positive the right first
      ear_state = EAR_COM_RIGHT;
    }
    else if(sound_LR_timer_diff <= -time_diff_min_mus){
      // if negative, then left first
      ear_state = EAR_COM_LEFT;
    }
    else{
      // if in the uncertainty range then go forward
      ear_state = EAR_COM_FORWARD;
    }
  }
  else if(found_sound_left || found_sound_right){
    if(found_sound_left){
      ear_state = EAR_COM_LEFT;
    }
    else if(found_sound_right){
      ear_state = EAR_COM_RIGHT;
    }
    else{
      // If only one ear is tripped then go forward
      ear_state = EAR_COM_FORWARD;
    }
  }
  else{
    ear_state = EAR_COM_NOSOUND;
  }

  if((millis()-ear_prev_reset_time) > ear_reset_time){
    ear_prev_reset_time = millis();

    #ifdef DEBUG_STATE
    Serial.print("STATE: ");
    Serial.print(ear_state);
    Serial.print(", FL: ");
    Serial.print(found_sound_left);
    Serial.print(", FR: ");
    Serial.print(found_sound_right);
    Serial.print(", TDiff: ");
    Serial.print(sound_LR_timer_diff);
    Serial.print(", M: ");
    if(ear_state==EAR_COM_FORWARD){Serial.print("F");}
    else if(ear_state==EAR_COM_LEFT){Serial.print("L");}
    else if(ear_state==EAR_COM_RIGHT){Serial.print("R");}
    else if(ear_state==EAR_COM_SENV){Serial.print("E");}
    else{Serial.print("N");}
    Serial.println();
    #endif

    // Reset Vars
    found_sound_both = false;
    found_sound_left = false;
    found_sound_right = false;
    sound_time_stamp_left = 0;
    sound_time_stamp_right = 0;
    sound_LR_timer_diff = 0;
  }
}
