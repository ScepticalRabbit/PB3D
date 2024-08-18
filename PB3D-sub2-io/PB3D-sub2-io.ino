//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Wire.h>
#include "RFDataSenderTX.h"
#include "Timer.h"

// Address for this board
#define NERVSYS_ADDR 9

//----------------------------------------------------------------------------
// BUMPER VARS
// Pins for collision sensors
#define COL_BUMPER_FL 0
#define COL_BUMPER_FR 1

byte collisionFlags = B00000000;

// Bumper Timer
uint16_t _bumperInt = 100; // ms
Timer _bumper_timer = Timer();

//----------------------------------------------------------------------------
// RF TX VARS
// This class owns the data packet
bool newPacket = false;
RFDataSenderTX sender;

// Timer for printing the data packet for diagnostics
bool _printDataOn = true;
Timer _printTimer = Timer();
uint16_t _printTime = STATEDATA_UPD_TIME; // ms

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  //while(!Serial){delay(10);}
  // Delays used for diagnostic purposes to see sender print to serial
  //delay(1000);
  sender.begin();
  //delay(1000);

  pinMode(COL_BUMPER_FR, INPUT_PULLUP);
  pinMode(COL_BUMPER_FL, INPUT_PULLUP);

  _bumper_timer.start(0);
  _printTimer.start(0);

  Wire.begin(NERVSYS_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

//----------------------------------------------------------------------------
// I2C HANDLER FUNCTIONS
void requestEvent(){
  Wire.write(collisionFlags);
}

void receiveEvent(int bytesRec){
  int16_t ii = 0;
  while(0<Wire.available()){
    if(ii < PACKET_SIZE){
      //_currState.dataPacket[ii] = Wire.read();
      sender.setStateByte(Wire.read(),ii);
    }
    else{
      Wire.read();
    }
    ii++;
  }
  if(ii >= PACKET_SIZE){
    sender.setNewPacket(true);
  }
}

void loop(){
  //---------------------------------------------------------------------------
  // BUMPERS - Send bumper flags when requested
  if(_bumper_timer.finished()){
    _bumper_timer.start(_bumperInt);
    byte temp = B00000000;

    int8_t switchFL = digitalRead(COL_BUMPER_FL);
    int8_t switchFR = digitalRead(COL_BUMPER_FR);

    // Note switches close, allow current to flow, voltage 0
    if(switchFL == 0){
      temp = temp | B00000001;
    }
    if(switchFR == 0){
      temp = temp | B00000010;
    }
    collisionFlags = temp;

    // Print flags to serial for debugging
    //Serial.print(F("COL FLAGS:"));
    //Serial.println(collisionFlags,BIN);
  }

  //---------------------------------------------------------------------------
  // RADIO - Receive data structure from main board by I2C
  sender.update();

  if(_printDataOn){
    if(_printTimer.finished()){
      _printTimer.start(_printTime);

      Serial.println(F("I2C DATA STRUCT REC:"));
      sender.printStateData();
    }
  }
}
