#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
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

#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

#include "src/CollisionManager.h"
#include "src/Encoder.h"
#include "src/FilterLowPass.h"
#include "src/FilterMovAvg.h"
#include "src/MoodManager.h"
#include "src/MoveManager.h"
#include "src/PatSensor.h"
#include "src/PID.h"
#include "src/Speaker.h"
#include "src/Tail.h"
#include "src/TaskManager.h"
#include "src/Timer.h"
#include "src/I2CDataSender.h"
#include "src/IMUSensor.h"
#include "src/Navigation.h"
#include "src/TaskDance.h"
#include "src/TaskRest.h"
#include "src/TaskTantrum.h"
#include "src/TaskInteract.h"
#include "src/TaskFindHuman.h"
#include "src/TaskFindLight.h"
#include "src/TaskFindSound.h"
#include "src/TaskPickedUp.h"
#include "src/TaskPounce.h"
#include "src/TaskPause.h"

//------------------------------------------------------------------------------
// VARIABLES

// Debugging - Codes
bool _debug_collisionOff = false;
bool _debug_forceMood = true;
EMoodCode _debug_moodCode = MOOD_NEUTRAL;
bool _debug_forceTask = true;
ETaskCode _debug_taskCode = TASK_EXPLORE;
bool _debug_forceMove = false;
int8_t _debug_moveType = MOVE_C_CIRCLE;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST VARIABLES
bool _test_switch = true;
uint16_t _test_time= 2000;
Timer _test_timer = Timer();

bool _test_retest = true;

float _test_value = 360.0; // Can also set to float
//uint8_t _test_value = 80;

uint16_t _test_reportTime = 200;
Timer _test_reportTimer = Timer();

bool _test_pauseSwitch = true;
uint8_t _test_count = 0;
uint8_t _test_countLim = 4;
uint32_t _test_pauseTime = 2000;
Timer _test_pauseTimer = Timer();

bool _test_firstLoop = true;

uint32_t _test_timeStamp = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//------------------------------------------------------------------------------
// EXTERNAL CLASSES

// MOOD LEDs
Adafruit_NeoPixel_ZeroDMA leds = Adafruit_NeoPixel_ZeroDMA(
  MOOD_NUMPIX, MOOD_PIN, NEO_GRB + NEO_KHZ800);

// MOTORS - Create the motor shield object with the default I2C address
Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

//------------------------------------------------------------------------------
// INTERNAL CLASSES

// SENSORS AND CONTROLLERS
static const int encoder_pinA_left = 2;
static const int encoder_pinB_left = 3;
static const int encoder_pinA_right = 4;
static const int encoder_pinB_right = 5;
// NOTE: encoders must be in main to attach interrupts
Encoder encoder_left = Encoder(encoder_pinA_left,encoder_pinB_left);
Encoder encoder_right = Encoder(encoder_pinA_right,encoder_pinB_right);

// BASIC CLASSES
MoveManager move_manager = MoveManager(&motor_shield,
                                       &encoder_left,
                                       &encoder_right);
MoodManager mood_manager = MoodManager(&leds);
TaskManager task_manager = TaskManager(&leds);
CollisionManager collision_manager = CollisionManager(&mood_manager,
                                                      &task_manager,
                                                      &move_manager);

// Sensors, Actuators and Communications
Speaker speaker = Speaker();
PatSensor pat_sensor = PatSensor();
Tail tail = Tail();
IMUSensor IMU = IMUSensor();
Navigation navigator = Navigation(&encoder_left,&encoder_right,&IMU);
I2CDataSender sender = I2CDataSender(&collision_manager,
                                     &mood_manager,
                                     &task_manager,
                                     &move_manager,
                                     &IMU,
                                     &navigator);

// TASK CLASSES - Uses basic classes
TaskDance task_dance = TaskDance(&mood_manager,
                                 &task_manager,
                                 &move_manager,
                                 &speaker);
TaskRest task_rest = TaskRest(&mood_manager,
                              &task_manager,
                              &move_manager,
                              &speaker);
TaskTantrum task_tantrum = TaskTantrum(&mood_manager,
                                       &task_manager,
                                       &move_manager,
                                       &speaker);
TaskInteract task_interact = TaskInteract(&mood_manager,
                                          &task_manager,
                                          &move_manager,
                                          &speaker,
                                          &task_dance,
                                          &pat_sensor);
TaskFindHuman task_find_human = TaskFindHuman(&mood_manager,
                                              &task_manager,
                                              &move_manager,
                                              &speaker,
                                              &task_interact);
TaskFindLight task_find_light = TaskFindLight(&mood_manager,
                                              &task_manager,
                                              &move_manager,
                                              &speaker,
                                              &pat_sensor);
TaskFindSound task_find_sound = TaskFindSound(&mood_manager,
                                              &task_manager,
                                              &move_manager,
                                              &speaker);
TaskPickedUp task_picked_up = TaskPickedUp(&collision_manager,
                                            &mood_manager,
                                            &task_manager,
                                            &move_manager,
                                            &speaker,
                                            &pat_sensor);
TaskPounce task_pounce = TaskPounce(&collision_manager,
                                    &mood_manager,
                                    &task_manager,
                                    &move_manager,
                                    &speaker);
TaskPause task_pause = TaskPause(&collision_manager,
                                 &task_manager,
                                 &move_manager,
                                 &speaker);

//-----------------------------------------------------------------------------
// SETUP
#line 175 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void setup();
#line 269 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void loop();
#line 448 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void updateEncLA();
#line 451 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void updateEncLB();
#line 454 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void updateEncRA();
#line 457 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void updateEncRB();
#line 463 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void escapeCollision();
#line 475 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void detectedCollision();
#line 175 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
void setup() {
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  // while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin();  // Join I2C bus as leader
  delay(2000);   // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println(F("PB3D: SETUP"));
  Serial.println(F("------------------------------"));

  // Seed the random generator - used by many classes 'begin' function
  randomSeed(analogRead(A1));

  leds.begin();
  leds.show();

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // TEST CODE

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // NOTE: I2C multiplexer and light sensors must be initialised first!
  task_find_light.begin();

  mood_manager.begin();
  task_manager.begin();
  move_manager.begin();
  collision_manager.begin();

  speaker.begin();
  pat_sensor.begin();
  tail.begin();
  IMU.begin();
  navigator.begin();
  sender.begin();

  task_find_human.begin();
  task_interact.begin();
  task_picked_up.begin();
  task_pounce.begin();
  task_pause.begin();

  // Make sure all the laser sensors are left on
  task_find_sound.setSendByte(B00111110);
  task_find_sound.begin();

  // Pass durations to the master task object
  task_manager.set_dance_duration(task_dance.getDuration());
  task_manager.set_tantrum_duration(task_tantrum.getDuration());

  // Start timers in the main program
  _test_timer.start(0);
  _test_reportTimer.start(0);
  _test_pauseTimer.start(_test_pauseTime);

  // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
  attachInterrupt(digitalPinToInterrupt(encoder_pinA_left),
                  updateEncLA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_pinA_right),
                  updateEncRA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_pinB_left),
                  updateEncLB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_pinB_right),
                  updateEncRB,CHANGE);

  //-------------------------------------------------------------------------
  // DEBUG MODE:
  if(_debug_forceMood){
    mood_manager.set_mood(_debug_moodCode);
  }
  if(_debug_forceTask){
    task_manager.set_task(_debug_taskCode);
  }
  if(_debug_forceMove){
    move_manager.update_move(_debug_moveType);
  }
  task_manager.assign_probability(mood_manager.get_mood());

  tail.setState(TAIL_CENT);
  Serial.println();

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

  //while(1){}
}

//------------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  //if(!_test_firstLoop){while(true){};}
  //uint32_t start_loop = millis();
  uint32_t start_loop = micros();

  //----------------------------------------------------------------------------
  // UPDATE MOOD - Based on internal timer, see Mood class
  mood_manager.update();

  // If mood has updated then modify the other classes
  if(mood_manager.get_new_mood_flag()){
    mood_manager.set_new_mood_flag(false); // Reset the flag
    task_manager.assign_probability(mood_manager.get_mood());
    move_manager.set_power_by_diff(mood_manager.get_power_diff());
    move_manager.set_speed_by_mood_fact(mood_manager.get_speed_fact());
  }
  if(_debug_forceMood){mood_manager.set_mood(_debug_moodCode);}

  //----------------------------------------------------------------------------
  // GENERATE TASK - Based on internal timer, see Task class
  task_manager.update();

  // If task has changed update mood LEDs
  if(task_manager.get_new_task_flag()){mood_manager.reset_mood();}

  // DEBUG MODE: force task to a particular value
  if(_debug_forceTask){
    if(task_manager.get_task() != _debug_taskCode){
      task_manager.set_task(_debug_taskCode);
    }
  }
  if(_debug_collisionOff){collision_manager.set_enabled_flag(false);}

  //----------------------------------------------------------------------------
  // UPDATE OBJECTS - Each object has own internal timer
  uint32_t startUpdate = micros();
  collision_manager.update();
  uint32_t endUpdate = micros();

  speaker.update();
  pat_sensor.update();
  tail.update();
  IMU.update();
  encoder_left.update_speed();
  encoder_right.update_speed();
  navigator.update();
  sender.update();

  task_dance.update();
  task_find_human.update();
  task_find_light.update();
  task_find_sound.update();
  task_interact.update();
  task_pause.update();
  task_picked_up.update();
  task_rest.update();

  // After updating all objects reset the new task flag
  task_manager.set_new_task_flag(false);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // TEST CODE - SENSOR REPORTS
  if(_test_reportTimer.finished()){
    _test_reportTimer.start(_test_reportTime);
    //DEBUG_PrintColCheck();
    //DEBUG_PrintAllRanges();
    //DEBUG_PrintLightSens();
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //----------------------------------------------------------------------------
  // RESET FLAGS
  // Reset the sound code and let the decision tree update it for the next loop
  uint8_t inCodes[] = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
  speaker.setSoundCodes(inCodes,4);
  // Reset the tail to the central position - allow tasks and mood control
  tail.setState(TAIL_CENT);

  //----------------------------------------------------------------------------
  // MAIN DECISION TREE
  //
  // NOTE: Rest and Interact are placed before collision avoidance to disable
  // handling of collisions while these modes are active - should be able to
  // make this smarter so certain things can re-enable collision avoidance
  if(task_manager.get_task() == TASK_TEST){
    //DEBUG_SpeedTest(_test_value,MOVE_B_FORWARD);
  }
  else if(task_manager.get_task() == TASK_REST){
    task_rest.rest();
  }
  else if(task_manager.get_task() == TASK_PAUSE){
    task_pause.pause();
  }
  else if(task_manager.get_task() == TASK_PICKEDUP){
    task_picked_up.pickedUp();
  }
  else if(task_manager.get_task() == TASK_INTERACT){
    task_interact.interact();
  }
  else if(collision_manager.get_escape_flag() && !_debug_collisionOff){
    //Serial.println("======================ESCAPE======================");
    escapeCollision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(collision_manager.get_detected() && !_debug_collisionOff){
    //DEBUG_PrintColFlags();
    detectedCollision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(task_manager.get_task() == TASK_DANCE){
    if(task_manager.get_dance_update_flag()){
      task_manager.set_dance_update_flag(false);
      task_dance.setSpeakerFlag(false);
    }
    task_dance.dance();
  }
  else if(task_manager.get_task() == TASK_FINDHUMAN){
    task_find_human.findHuman();
  }
  else if(task_manager.get_task() == TASK_FINDLIGHT){
    task_find_light.findLight();
  }
  else if(task_manager.get_task() == TASK_FINDDARK){
    task_find_light.findDark();
  }
  else if(task_manager.get_task() == TASK_FINDSOUND){
    task_find_sound.findSound();
  }
  else if(task_manager.get_task() == TASK_POUNCE){
    task_pounce.seek_and_pounce();
  }
  else if(task_manager.get_task() == TASK_TANTRUM){
    if(task_tantrum.getCompleteFlag()){
      task_manager.force_update();
    }
    else{
      task_tantrum.haveTantrum();
    }
  }
  else{
    task_manager.task_LED_explore();

    // MOVEMENT: Type Update
    if(_debug_forceMove){
      move_manager.update_move(_debug_moveType);
    }
    else{
      move_manager.update_move();
    }
    // MOVEMENT: Call current movement function
    move_manager.go();
  }

  //-------------------------------------------------------------------------
  // POST DECISION TREE OVERRIDES
  // Wag tail if happy - regardless of task use of tail
  if(mood_manager.get_mood() == MOOD_HAPPY){
    tail.setState(TAIL_WAG_INT);
    // wagmovetime,wagoffset,_test_pauseTime,wagcount
    tail.setWagParams(200,30,4000,6);
  }

  // If sleeping don't wag tail
  if(task_manager.get_task() == TASK_REST){
    tail.setState(TAIL_CENT);
  }

  //-------------------------------------------------------------------------
  // END of LOOP

  //uint32_t endLoop = millis();
  //Serial.print(F("MAIN LOOP TOOK: "));
  //Serial.print(endLoop-start_loop); Serial.print(",");
  //Serial.print(endUpdate-startUpdate); Serial.print(",");
  //Serial.println();
  //_test_firstLoop = false;

}

//------------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
void updateEncLA(){
  encoder_left.update_not_equal();
}
void updateEncLB(){
  encoder_left.update_equal();
}
void updateEncRA(){
  encoder_right.update_equal();
}
void updateEncRB(){
  encoder_right.update_not_equal();
}

//------------------------------------------------------------------------------
// COLLISION HANDLING TASK TREE FUNCTIONS
void escapeCollision(){
  task_manager.task_LED_collision();
  collision_manager.reset_flags();

  if(collision_manager.get_beepbeep_flag()){
    uint8_t inCodes[] = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speaker.setSoundCodes(inCodes,4);
  }

  collision_manager.escape();
}

void detectedCollision(){
  // Turn on collision LEDs and set escape flags
  task_manager.task_LED_collision();
  collision_manager.set_escape_start();

  // If we are moving in a circle or a spiral then switch direction
  move_manager.change_circ_dir();

  // Reset the PIDs and stop the motors
  move_manager.stop();

  // If the bumper flag was tripped we need to go beep,beep!
  collision_manager.set_beepbeep_flag(false);
  if(collision_manager.get_bumper_flag()){
    collision_manager.set_beepbeep_flag(true);

    speaker.reset();
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speaker.setSoundCodes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_G4,NOTE_FS4,NOTE_G4,NOTE_FS4,0,0,0,0};
    speaker.setSoundFreqs(inFreqs,8);
    uint16_t inDurs[]   = {200,100,200,100,0,0,0,0};
    speaker.setSoundDurs(inDurs,8);
  }

  // Call specific tasks that need to handle collision events
  if((task_manager.get_task() == TASK_FINDLIGHT)||(task_manager.get_task() == TASK_FINDDARK)){
    task_find_light.resetGrad();
  }
  if((task_manager.get_task() == TASK_POUNCE) && (task_pounce.get_state() == POUNCE_RUN)){
    task_pounce.collision_reset_to_realign();

    int16_t angCent = task_pounce.get_ang_cent_for_collision();
    if(collision_manager.get_escape_turn() == MOVE_B_LEFT){
      task_pounce.set_realign_cent(-1*angCent);
    }
    else{
      task_pounce.set_realign_cent(1*angCent);
    }
  }

  // Reset the collision flags
  collision_manager.reset_flags();
  collision_manager.inc_count();
  if(collision_manager.get_count() >= task_tantrum.getThreshold()){
    collision_manager.reset_Count();
    task_tantrum.setStartTantrumFlag();
    task_manager.set_task(TASK_TANTRUM);
  }
}

//------------------------------------------------------------------------------
// DEBUG FUNCTIONS
/*
void DEBUG_SpeedTest(uint8_t inPower, uint8_t moveCode){
  if(_test_pauseTimer.finished()){
    if(_test_pauseSwitch){
      _test_pauseSwitch = false;
      _test_timer.start(_test_time);
      _test_switch = true;
    }
    DEBUG_PlotSpeedMMPS();

    if(_test_timer.finished()){
      if(_test_retest){
        _test_timer.start(_test_time);
        _test_switch = !_test_switch;
      }
      else{
        _test_switch = false;
      }
    }

    if(_test_switch){
      if(moveCode == MOVE_B_BACK){
        move_manager.back_power(inPower);
      }
      else if(moveCode == MOVE_B_LEFT){
        move_manager.left_power(inPower);
      }
      else if(moveCode == MOVE_B_RIGHT){
        move_manager.right_power(inPower);
      }
      else{
        move_manager.forward_power(inPower);
      }
    }
    else{
      move_manager.stop();
    }
  }
  else{
    move_manager.stop();
  }
}

void DEBUG_SpeedTest(float inSpeed, uint8_t moveCode){
  if(_test_pauseTimer.finished()){
    if(_test_pauseSwitch){
      _test_pauseSwitch = false;
      _test_timer.start(_test_time);
      _test_switch = true;
    }
    DEBUG_PlotSpeedMMPS();

    if(_test_timer.finished()){
      if(_test_retest){
        _test_timer.start(_test_time);
        _test_switch = !_test_switch;
      }
      else{
        _test_switch = false;
      }
    }

    if(_test_switch){
      if(moveCode == MOVE_B_BACK){
        move_manager.back(inSpeed);
      }
      else if(moveCode == MOVE_B_LEFT){
        move_manager.left(inSpeed);
      }
      else if(moveCode == MOVE_B_RIGHT){
        move_manager.right(inSpeed);
      }
      else{
        move_manager.forward(inSpeed);
      }
    }
    else{
      move_manager.stop();
    }
  }
  else{
    move_manager.stop();
  }
}

void DEBUG_PrintSpeedMMPS(){
  Serial.println();
  Serial.print(F("ENCODER SPEED, L="));
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(F(" mm/s, R = "));
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(F(" mm/s"));
  Serial.println();
}

void DEBUG_PrintSpeedCPS(){
  Serial.println();
  Serial.print(F("ENCODER SPEED, L="));
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(F(" mm/s, R = "));
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(F(" mm/s"));
  Serial.println();
}

void DEBUG_PlotSpeedBoth(){
  Serial.print(encoder_left.get_smooth_speed_cps());
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.print(encoder_right.get_smooth_speed_cps());
  Serial.print(",");
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedPID_L(){
  Serial.print(move_manager.get_speed_PID_set_point_left());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_output_left());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Pterm_left());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Iterm_left());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Dterm_left());
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedPID_R(){
  Serial.print(move_manager.get_speed_PID_set_point_right());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_output_right());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Pterm_right());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Iterm_right());
  Serial.print(",");
  Serial.print(move_manager.get_speed_PID_Dterm_right());
  Serial.print(",");
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedMMPS(){
  uint32_t thisTime = millis();
  Serial.print(thisTime-_test_timeStamp);
  _test_timeStamp = thisTime;
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PrintColCheck(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println("CheckVec=[BL,BR,US,LL,LR,LU,LD,]");
  Serial.print("CheckVec=[");
  for(uint8_t ii=0;ii<7;ii++){
      Serial.print(" ");Serial.print(collision_manager.get_col_check(ii));Serial.print(",");
  }
  Serial.println("]");

  Serial.print(F("US: "));
  //Serial.print(collision_manager.getColUSFlag());
  Serial.print(F(" , "));
  Serial.print(collision_manager.get_ultrasonic_range_mm());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,L: "));
  //Serial.print(collision_manager.getColLSRFlagL());
  Serial.print(F(" , "));
  Serial.print(collision_manager.getLSRRangeL());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,R: "));
  //Serial.print(collision_manager.getColLSRFlagR());
  Serial.print(F(" , "));
  Serial.print(collision_manager.getLSRRangeR());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,A: "));
  //Serial.print(collision_manager.getColLSRFlagB());
  Serial.print(F(" , "));
  Serial.print(collision_manager.getLSRRangeA());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,U: "));
  //Serial.print(collision_manager.getColLSRFlagU());
  Serial.print(F(" , "));
  Serial.print(collision_manager.getLSRRangeU());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,D: "));
  //Serial.print(collision_manager.getColLSRFlagD());
  Serial.print(F(" , "));
  Serial.print(collision_manager.getLSRRangeD());
  Serial.println(F(" mm"));

  Serial.println(F("------------------------------"));
  Serial.println();
}//---------------------------------------------------------------------------

void DEBUG_PrintAllRanges(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.print(F("US: "));
  Serial.print(collision_manager.get_ultrasonic_range_mm());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,L: "));
  Serial.print(collision_manager.getLSRRangeL());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,R: "));
  Serial.print(collision_manager.getLSRRangeR());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,A: "));
  Serial.print(collision_manager.getLSRRangeA());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,U: "));
  Serial.print(collision_manager.getLSRRangeU());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,D: "));
  Serial.print(collision_manager.getLSRRangeD());
  Serial.println(F(" mm"));

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_PrintLightSens(){
  Serial.print("Light Sens L-");
  Serial.print("Lux: "); Serial.print(task_find_light.getLuxLeft());
  Serial.print(", R-");
  Serial.print("Lux: "); Serial.println(task_find_light.getLuxRight());
}

*/

