//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_NeoPixel_ZeroDMA.h>

#include <PB3DConstants.h>
#include <PB3DPins.h>
#include <PB3DI2CAddresses.h>
#include <PB3DTimer.h>

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
#include "src/I2CDataSender.h"
#include "src/IMUSensor.h"
#include "src/MultiIOExpander.h"
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
// DEBUG VARIABLES
bool debug_collision_off = false;
bool debug_force_mood = true;
EMoodCode debug_mood_code = MOOD_NEUTRAL;
bool debug_force_task = true;
ETaskCode debug_taskCode = TASK_EXPLORE;
bool debug_force_move = false;
EMoveCompound debug_move_type = MOVE_C_CIRCLE;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST VARIABLES
bool test_switch = true;
uint16_t test_time= 1000;
Timer test_timer = Timer();

bool test_retest = true;

float test_value = 360.0; // Can also set to float
//uint8_t test_value = 80;

uint16_t test_report_time = 100;
Timer test_report_timer = Timer();

bool _test_pause_switch = true;
uint8_t test_count = 0;
uint8_t test_count_limit = 4;
uint32_t test_pause_time = 2000;
Timer test_pause_timer = Timer();

bool test_first_loop = true;

uint32_t test_time_stamp = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//------------------------------------------------------------------------------
// EXTERNAL CLASSES

// MOOD / TASK LEDs - required by mood and task manager
Adafruit_NeoPixel_ZeroDMA leds = Adafruit_NeoPixel_ZeroDMA(
            NUM_PIX, MOOD_TASK_LED_PIN, NEO_GRB + NEO_KHZ800);

//------------------------------------------------------------------------------
// INTERNAL CLASSES

// NOTE: encoders must be in main to attach interrupts
Encoder encoder_left = Encoder(ENCODER_PINA_LEFT,ENCODER_PINB_LEFT);
Encoder encoder_right = Encoder(ENCODER_PINA_RIGHT,ENCODER_PINB_RIGHT);

// Core managers for decision engine: mood determines task probability
MoveManager move_manager = MoveManager(&encoder_left,
                                       &encoder_right);
MoodManager mood_manager = MoodManager(&leds);
TaskManager task_manager = TaskManager(&leds);

// Sensors and Actuators
MultiIOExpander multi_expander = MultiIOExpander(GPIO_PIN_MODES);
LaserManager laser_manager = LaserManager(&multi_expander);
BumperSensor bumpers = BumperSensor(&multi_expander);
Speaker speaker = Speaker();
PatSensor pat_sensor = PatSensor();
Tail tail = Tail();
IMUSensor IMU = IMUSensor();
Navigation navigator = Navigation(&encoder_left,&encoder_right,&IMU);

// Collision Detection & Escape
CollisionManager collision_manager = CollisionManager(&mood_manager,
                                                      &task_manager,
                                                      &move_manager,
                                                      &laser_manager,
                                                      &bumpers);

// Communications
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
                                            &pat_sensor,
                                            &multi_expander);
TaskPounce task_pounce = TaskPounce(&collision_manager,
                                    &mood_manager,
                                    &task_manager,
                                    &move_manager,
                                    &speaker);
TaskPause task_pause = TaskPause(&collision_manager,
                                 &task_manager,
                                 &move_manager,
                                 &speaker);

//==============================================================================
void setup() {
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  // while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin();  // Join I2C bus as leader
  delay(2000);   // Needed to ensure sensors work, delay allows sub-processors to start up

  Serial.println();
  Serial.println(F("PB3D: SETUP"));
  Serial.println(F("------------------------------"));

  // Seed the random generator - used by many classes 'begin' function
  randomSeed(analogRead(A1));

  leds.begin();
  leds.show();

  // NOTE: I2C multiplexer and light sensors MUST be initialised first!
  task_find_light.begin();
  multi_expander.begin();
  laser_manager.begin();
  bumpers.begin();

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
  task_find_sound.set_send_byte(B00111110);
  task_find_sound.begin();

  task_manager.set_dance_duration(task_dance.get_duration());
  task_manager.set_tantrum_duration(task_tantrum.get_duration());

  test_timer.start(0);
  test_report_timer.start(0);
  test_pause_timer.start(test_pause_time);

  // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA_LEFT),
                  update_encoder_left_pina,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA_RIGHT),
                  update_encoder_right_pina,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB_LEFT),
                  update_encoder_left_pinb,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB_RIGHT),
                  update_encoder_right_pinb,CHANGE);

  //-------------------------------------------------------------------------
  // DEBUG MODE:
  if(debug_force_mood){
    mood_manager.set_mood(debug_mood_code);
  }
  if(debug_force_task){
    task_manager.set_task(debug_taskCode);
  }
  if(debug_force_move){
    move_manager.update(debug_move_type);
  }
  task_manager.assign_probability(mood_manager.get_mood());

  tail.set_state(TAIL_CENT);
  Serial.println();

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

  // while(1){}
} // END SETUP

//==============================================================================
void loop(){
  //if(!test_first_loop){while(true){};}
  //uint32_t start_loop = millis();
  uint32_t start_loop = micros();

  //----------------------------------------------------------------------------
  // UPDATE MOOD - Based on internal timer, see Mood class
  mood_manager.update();

  // If mood has updated then modify the other classes
  if(mood_manager.get_new_mood_flag()){
    mood_manager.set_new_mood_flag(false); // Reset the flag
    task_manager.assign_probability(mood_manager.get_mood());

    move_manager.set_speed_mood_multiplier(mood_manager.get_speed_fact());
  }
  if(debug_force_mood){mood_manager.set_mood(debug_mood_code);}

  //----------------------------------------------------------------------------
  // GENERATE TASK - Based on internal timer, see Task class
  task_manager.update();

  // If task has changed update mood LEDs
  if(task_manager.get_new_task_flag()){mood_manager.reset_mood();}

  // DEBUG MODE: force task to a particular value
  if(debug_force_task){
    if(task_manager.get_task() != debug_taskCode){
      task_manager.set_task(debug_taskCode);
    }
  }
  if(debug_collision_off){collision_manager.enabled = false;}

  //----------------------------------------------------------------------------
  // UPDATE OBJECTS - Each object has own internal timer
  laser_manager.update();
  bumpers.update();
  encoder_left.update_speed();
  encoder_right.update_speed();

  move_manager.update(); // TODO: check this doesn't override tasks
  collision_manager.update(); // NOTE: collision latches set in here

  speaker.update();
  pat_sensor.update();
  tail.update();
  IMU.update();
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
  if(test_report_timer.finished()){
    test_report_timer.start(test_report_time);
    DEBUG_print_encoders();
    //DEBUG_col_check();
    //DEBUG_ranges();
    //DEBUG_col_with_ranges();
    //DEBUG_bumpers();
    //DEBUG_light_sens();
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //----------------------------------------------------------------------------
  // RESET FLAGS
  // Reset the sound code and let the decision tree update it for the next loop
  uint8_t inCodes[] = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
  speaker.set_sound_codes(inCodes,4);
  // Reset the tail to the central position - allow tasks and mood control
  tail.set_state(TAIL_CENT);

  //----------------------------------------------------------------------------
  // MAIN DECISION TREE
  //
  // NOTE: Rest and Interact are placed before collision avoidance to disable
  // handling of collisions while these modes are active - should be able to
  // make this smarter so certain things can re-enable collision avoidance
  if(task_manager.get_task() == TASK_TEST){
    DEBUG_speed_test(80.0,MOVE_B_FORWARD);
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
  else if(collision_manager.get_escape_state() && !debug_collision_off){
    escape_collision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(collision_manager.get_detected() && !debug_collision_off){
    DEBUG_col_check();
    detected_collision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(task_manager.get_task() == TASK_DANCE){
    if(task_manager.get_dance_update_flag()){
      task_manager.set_dance_update_flag(false);
      task_dance.set_speaker_flag(false);
    }
    task_dance.dance();
  }
  else if(task_manager.get_task() == TASK_FINDHUMAN){
    task_find_human.find_human();
  }
  else if(task_manager.get_task() == TASK_FINDLIGHT){
    task_find_light.find_light();
  }
  else if(task_manager.get_task() == TASK_FINDDARK){
    task_find_light.find_dark();
  }
  else if(task_manager.get_task() == TASK_FINDSOUND){
    task_find_sound.find_sound();
  }
  else if(task_manager.get_task() == TASK_POUNCE){
    task_pounce.seek_and_pounce();
  }
  else if(task_manager.get_task() == TASK_TANTRUM){
    if(task_tantrum.get_complete()){
      task_manager.force_update();
    }
    else{
      task_tantrum.chuck_tantrum();
    }
  }
  else{
    task_manager.task_LED_explore();

    // MOVEMENT: Type Update
    if(debug_force_move){
      move_manager.update(debug_move_type);
    }
    else{
      move_manager.update();
    }
    // MOVEMENT: Call current movement function
    move_manager.go();
  }

  //-------------------------------------------------------------------------
  // POST DECISION TREE OVERRIDES
  // Wag tail if happy - regardless of task use of tail
  if(mood_manager.get_mood() == MOOD_HAPPY){
    tail.set_state(TAIL_WAG_INT);
    // wagmovetime,wagoffset,test_pause_time,wagcount
    tail.set_wag_params(200,30,4000,6);
  }

  // If sleeping don't wag tail
  if(task_manager.get_task() == TASK_REST){
    tail.set_state(TAIL_CENT);
  }

  //-------------------------------------------------------------------------
  // END of LOOP
  //uint32_t endLoop = millis();
  //Serial.print(F("MAIN LOOP TOOK: "));
  //Serial.print(endLoop-start_loop); Serial.print(",");
  //Serial.print(end_update-start_update); Serial.print(",");
  //Serial.println();
  //test_first_loop = false;

} // END LOOP
//==============================================================================

//------------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
void update_encoder_left_pina(){
  encoder_left.update_not_equal();
}
void update_encoder_left_pinb(){
  encoder_left.update_equal();
}
void update_encoder_right_pina(){
  encoder_right.update_equal();
}
void update_encoder_right_pinb(){
  encoder_right.update_not_equal();
}

//------------------------------------------------------------------------------
// COLLISION HANDLING TASK TREE FUNCTIONS
void escape_collision(){
  task_manager.task_LED_collision();
  collision_manager.reset_unlatch(); // NOTE: collision unlatch here

  if(collision_manager.get_beepbeep_flag()){
    uint8_t inCodes[] = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speaker.set_sound_codes(inCodes,4);
  }

  collision_manager.escape();
}

void detected_collision(){
  // Turn on collision LEDs and set escape flags
  task_manager.task_LED_collision();
  collision_manager.set_escape_start();

  // If we are moving in a circle or a spiral then switch direction
  move_manager.change_turn_dir();

  // Reset the PIDs and stop the motors
  move_manager.stop();

  // If the bumper flag was tripped we need to go beep,beep!
  collision_manager.set_beepbeep_flag(false);
  if(collision_manager.get_bumper_flag()){
    collision_manager.set_beepbeep_flag(true);

    speaker.reset();
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speaker.set_sound_codes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_G4,NOTE_FS4,NOTE_G4,NOTE_FS4,0,0,0,0};
    speaker.set_sound_freqs(inFreqs,8);
    uint16_t inDurs[]   = {200,100,200,100,0,0,0,0};
    speaker.set_sound_durations(inDurs,8);
  }

  // Call specific tasks that need to handle collision events
  if((task_manager.get_task() == TASK_FINDLIGHT)||(task_manager.get_task() == TASK_FINDDARK)){
    task_find_light.reset_gradient();
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
  collision_manager.reset_unlatch();
  collision_manager.inc_count();
  if(collision_manager.get_count() >= task_tantrum.get_threshold()){
    collision_manager.reset_Count();
    task_tantrum.set_start_tantrum();
    task_manager.set_task(TASK_TANTRUM);
  }
}

//==============================================================================
// DEBUG
// #define PB3D_DEBUG

void DEBUG_speed_test(float speed, uint8_t move_code){
  if(test_pause_timer.finished()){
    if(_test_pause_switch){
      _test_pause_switch = false;
      test_timer.start(test_time);
      test_switch = true;
    }

    if(test_timer.finished()){
      if(test_retest){
        test_timer.start(test_time);
        test_switch = !test_switch;
      }
      else{
        test_switch = false;
      }
    }

    if(test_switch){
      if(move_code == MOVE_B_BACK){
        move_manager.set_back_speed(speed);
        move_manager.back();
      }
      else if(move_code == MOVE_B_LEFT){
        move_manager.set_turn_speed(speed);
        move_manager.left();
      }
      else if(move_code == MOVE_B_RIGHT){
        move_manager.set_turn_speed(speed);
        move_manager.right();
      }
      else{
        move_manager.set_forward_speed(speed);
        move_manager.forward();
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

void DEBUG_print_encoders(){
  Serial.println();
  Serial.print(F("ENCODER SPEED, L="));
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(F(" mm/s, R = "));
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(F(" mm/s"));
  Serial.println();
  Serial.print(F("ENCODER COUNTS, L="));
  Serial.print(encoder_left.get_count());
  Serial.print(F(" , R = "));
  Serial.print(encoder_right.get_count());
  Serial.println();

}

void DEBUG_plot_speed(){
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
  Serial.print(move_manager.get_move_controller()->get_speed_PID_left()->get_set_point());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_left()->get_output());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_left()->get_prop_term());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_left()->get_int_term());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_left()->get_deriv_term());
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedPID_R(){
  Serial.print(move_manager.get_move_controller()->get_speed_PID_right()->get_set_point());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_right()->get_output());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_right()->get_prop_term());
  Serial.print(",");
  //Serial.print(move_manager.get_speed_PID_Iterm_right());
  Serial.print(",");
  Serial.print(move_manager.get_move_controller()->get_speed_PID_right()->get_deriv_term());
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_plot_speed_mmps(){
  uint32_t thisTime = millis();
  Serial.print(thisTime-test_time_stamp);
  test_time_stamp = thisTime;
  Serial.print(",");
  Serial.print(encoder_left.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.print(encoder_right.get_smooth_speed_mmps());
  Serial.print(",");
  Serial.println();
}

void DEBUG_bumpers(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println(F("DEBUG BUMPERS"));
  Serial.print("Bump, L = ");
  Serial.println(bumpers.get_collision_code(BUMP_LEFT));
  Serial.print("Bump, R = ");
  Serial.println(bumpers.get_collision_code(BUMP_RIGHT));
  Serial.print("Bump, B = ");
  Serial.println(bumpers.get_collision_code(BUMP_BACK));

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_col_check(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println(F("DEBUG COLLISION CHECK"));
  Serial.println(F("------------------------------"));
  Serial.print("Bump, L = ");
  Serial.println(bumpers.get_collision_code(BUMP_LEFT));
  Serial.print("Bump, R = ");
  Serial.println(bumpers.get_collision_code(BUMP_RIGHT));
  Serial.print("Bump, B = ");
  Serial.println(bumpers.get_collision_code(BUMP_BACK));

  for(uint8_t ll = 0; ll < LASER_COUNT; ll++){
    Serial.print("LASER, ");
    Serial.print(LASER_STRS[ll]);
    Serial.print(" = ");
    Serial.print(laser_manager.get_collision_code(ELaserIndex(ll)));
    Serial.println();
  }

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_ranges(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println(F("DEBUG RANGES CHECK"));
  Serial.println(F("------------------------------"));

  for(uint8_t ll = 0; ll < LASER_COUNT; ll++){
    Serial.print("LASER, ");
    Serial.print(LASER_STRS[ll]);
    Serial.print(" = ");
    Serial.print(laser_manager.get_range(ELaserIndex(ll)));
    Serial.print(" mm");
    Serial.println();
  }

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_col_with_ranges(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println(F("DEBUG COL & RANGES CHECK"));
  Serial.println(F("------------------------------"));

  for(uint8_t ll = 0; ll < LASER_COUNT; ll++){
    Serial.print("LASER, ");
    Serial.print(LASER_STRS[ll]);
    Serial.print(": C ");
    Serial.print(laser_manager.get_collision_code(ELaserIndex(ll)));
    Serial.print(" , R = ");
    Serial.print(laser_manager.get_range(ELaserIndex(ll)));
    Serial.print(" mm");
    Serial.println();
  }

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_light_sens(){
  Serial.print("Light Sens L-");
  Serial.print("Lux: "); Serial.print(task_find_light.get_lux_left());
  Serial.print(", R-");
  Serial.print("Lux: "); Serial.println(task_find_light.get_lux_right());
}

