//==============================================================================
// PB3D: A 3D printed pet robot
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
// RF TX VARS
// This class owns the data packet
bool new_packet = false;
RFDataSenderTX rf_sender;

// Timer for printing the data packet for diagnostics
bool print_data_on = true;
Timer print_timer = Timer();
uint16_t print_time = STATEDATA_UPD_TIME; // ms

//----------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  //while(!Serial){delay(10);}
  // Delays used for diagnostic purposes to see rf_sender print to serial
  //delay(1000);
  rf_sender.begin();
  //delay(1000);

  print_timer.start(0);

  Wire.begin(ADDR_FOLLOW_XIAO_2);
  Wire.onReceive(receive_event);
}

//------------------------------------------------------------------------------
void loop(){
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
