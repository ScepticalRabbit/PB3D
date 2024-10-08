//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS - RFLOGGERRX
//-----------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
Date Created: 10th Dec. 2022
Date Edited:  10th Dec. 2022
*/

#ifndef RFLOGGERRX_H
#define RFLOGGERRX_H

//----------------------------------------------------------------------------
// INCLUDES
// Include required classes for SPI and RF chip
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include "PB3DTimer.h"

//----------------------------------------------------------------------------
// DEFINITIONS
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.2

// SPI Radio Pinouts for the Xiao
#define RFM69_CS      7
#define RFM69_INT     3
#define RFM69_RST     6
#define LED           13

// This board is the client sending ackMsg to the server
#define CLIN_RF_ADDR     1

//----------------------------------------------------------------------------
// DATA STRUCTURES
// Declare data structure and union types
typedef struct stateData_t{
  uint8_t mood;
  uint8_t task;
  bool collisionFlags[4];
  float wheelSpeed;
};

typedef union radioPacket_t{
  stateData_t state;
  byte rfPacket[sizeof(stateData_t)];
};

#define PACKET_SIZE sizeof(stateData_t)

//----------------------------------------------------------------------------
// CLASS
class RFLoggerRX{
public:
  RFLoggerRX(){
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // RF: reset pin
    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    // RF: Reset the RF chip
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if (!_rf69Manager.init()) {
      Serial.println(F("RF RX: Failed to init RF RX"));
      while (1);
    }
    Serial.println(F("RF TX: initialised."));

    // RF: set parameters
    if (!_rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
    }
    _rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    // RF: Encryption
    uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                      0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
    _rf69.setEncryptionKey(key);

    Serial.print("RFM69 RX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

    // INIT CLASS:
    _curr_state.state.mood = 0;
    _curr_state.state.task = 0;
    _curr_state.state.collisionFlags[0] = false;
    _curr_state.state.collisionFlags[1] = false;
    _curr_state.state.collisionFlags[2] = false;
    _curr_state.state.collisionFlags[3] = false;
    _curr_state.state.wheelSpeed = 0.0;

    Serial.println(F("INITIAL DATA STRUCT"));
    printRFMsgStruct();
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(_rf69Manager.available()){
      uint8_t len = sizeof(_buf);
      uint8_t from;

      if (_rf69Manager.recvfromAck(_curr_state.rfPacket, &len, &from)) {
        _buf[len] = 0; // zero out remaining string

        Serial.println();
        Serial.print("Rec packet from #"); Serial.print(from);
        Serial.print(" [RSSI :"); Serial.print(_rf69.lastRssi()); Serial.print("] : ");

        Serial.println();
        Serial.println(F("REC DATA STRUCTURE:"));
        printRFMsgStruct();

        // Send a reply back to the originator client
        uint8_t ackMsg[] = "Data Rec.";
        if (!_rf69Manager.sendtoWait(ackMsg, sizeof(ackMsg), from)){
          Serial.println(F("Send failed (no ack)"));
        }
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

  //---------------------------------------------------------------------------
  // DIAGNOSTIC FUNCTIONS
  void printRFMsgStruct(){
    Serial.print(F("Mood: "));
    Serial.print(_curr_state.state.mood);
    Serial.print(F("; "));

    Serial.print(F("TaskManager: "));
    Serial.print(_curr_state.state.task);
    Serial.print(F("; "));

    Serial.print(F("Col Flags: "));
    for(uint8_t ii = 0; ii < 4; ii++){
      if(_curr_state.state.collisionFlags[ii]){
        Serial.print(F("1"));
      }
      else{
        Serial.print(F("0"));
      }
    }
    Serial.print(F("; "));

    Serial.print(F("Speed: "));
    Serial.print(_curr_state.state.wheelSpeed);
    Serial.print(F("; "));
    Serial.println();
  }

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;
  bool _start_flag = true;

  // Declare instance of the packet to send
  radioPacket_t _curr_state;

  // Radio class and radio ackMsg manager class
  // RH_RF69_MAX_MESSAGE_LEN = 60
  RH_RF69 _rf69 = RH_RF69(RFM69_CS, RFM69_INT);
  RHReliableDatagram _rf69Manager = RHReliableDatagram(_rf69, CLIN_RF_ADDR);
  int16_t _packetnum = 0;  // packet counter, we increment per xmission

  // Buffer and acknowledgement message
  uint8_t _buf[RH_RF69_MAX_MESSAGE_LEN];
};
#endif
