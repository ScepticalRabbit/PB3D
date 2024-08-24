#include <Arduino.h>
#line 1 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Wire.h>
#include <PB3DI2CAddresses.h>
#include <PB3DTimer.h>
#include "RFDataSenderTX.h"

//----------------------------------------------------------------------------
// BUMPER VARS
// Pins for collision sensors
enum EBumpPin{
    BUMP_DIN_LEFT = 0,
    BUMP_DIN_RIGHT = 1,
};

byte collision_flags = B00000000;

// Bumper Timer
uint16_t bumper_check_time = 100; // ms
Timer bumper_timer = Timer();

//----------------------------------------------------------------------------
// RF TX VARS
// This class owns the data packet
bool new_packet = false;
RFDataSenderTX rf_sender;

// Timer for printing the data packet for diagnostics
bool print_data_on = true;
Timer print_timer = Timer();
uint16_t print_time = STATEDATA_UPD_TIME; // ms

//----------------------------------------------------------------------------
#line 41 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
void setup();
#line 61 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
void loop();
#line 101 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
void request_event();
#line 105 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
void receive_event(int bytes_rec);
#line 41 "/home/lloydf/Arduino/PB3D/PB3D-sub2-iotx/PB3D-sub2-iotx.ino"
void setup(){
  Serial.begin(115200);
  //while(!Serial){delay(10);}
  // Delays used for diagnostic purposes to see rf_sender print to serial
  //delay(1000);
  rf_sender.begin();
  //delay(1000);

  pinMode(BUMP_DIN_RIGHT, INPUT_PULLUP);
  pinMode(BUMP_DIN_LEFT, INPUT_PULLUP);

  bumper_timer.start(0);
  print_timer.start(0);

  Wire.begin(ADDR_FOLLOW_XIAO_2);
  Wire.onRequest(request_event);
  Wire.onReceive(receive_event);
}

//------------------------------------------------------------------------------
void loop(){
  //----------------------------------------------------------------------------
  // BUMPERS - Send bumper flags when requested
  if(bumper_timer.finished()){
    bumper_timer.start(bumper_check_time);
    byte temp_byte = B00000000;

    int8_t switch_bump_left = digitalRead(BUMP_DIN_LEFT);
    int8_t switch_bump_right = digitalRead(BUMP_DIN_RIGHT);

    // Note switches close, allow current to flow, voltage 0
    if(switch_bump_left == 0){
      temp_byte = temp_byte | B00000001;
    }
    if(switch_bump_right == 0){
      temp_byte = temp_byte | B00000010;
    }
    collision_flags = temp_byte;

    // Print flags to serial for debugging
    //Serial.print(F("COL FLAGS:"));
    //Serial.println(collision_flags,BIN);
  }

  //---------------------------------------------------------------------------
  // RADIO - Receive data structure from main board by I2C
  rf_sender.update();

  if(print_data_on){
    if(print_timer.finished()){
      print_timer.start(print_time);

      Serial.println(F("I2C DATA STRUCT REC:"));
      rf_sender.printStateData();
    }
  }
}

//----------------------------------------------------------------------------
// I2C HANDLER FUNCTIONS
void request_event(){
  Wire.write(collision_flags);
}

void receive_event(int bytes_rec){
  int16_t ii = 0;
  while(0<Wire.available()){
    if(ii < PACKET_SIZE){
      //_curr_state.data_packet[ii] = Wire.read();
      rf_sender.set_state_byte(Wire.read(),ii);
    }
    else{
      Wire.read();
    }
    ii++;
  }
  if(ii >= PACKET_SIZE){
    rf_sender.set_new_packet(true);
  }
}

