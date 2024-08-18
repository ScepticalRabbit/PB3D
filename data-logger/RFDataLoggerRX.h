//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef RFDATALOGGERRX_H
#define RFDATALOGGERRX_H

//----------------------------------------------------------------------------
// INCLUDES
// Include required classes for SPI and RF chip
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

// Header for the state data packet
#include "StateData.h"

//----------------------------------------------------------------------------
// DEFINITIONS
//#define RFRX_DEBUG_PRINT
//#define RFRX_DEBUG_SPEED

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
// CLASS
class RFDataLoggerRX{
public:
  RFDataLoggerRX(){
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
      while(1);
    }
    Serial.println(F("RF TX: initialised."));

    // RF: set parameters
    if (!_rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
    }
    _rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    // RF: Encryption
    /*
    uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                      0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
    _rf69.setEncryptionKey(key);
    */

    Serial.print("RFM69 RX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

    Serial.println(F("INITIAL DATA STRUCT"));
    _initStateData(&_currState);
    _printStateData(&_currState);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(_rf69Manager.available()){
      uint8_t len = sizeof(_buf);
      uint8_t from;

      if (_rf69Manager.recvfromAck(_currState.dataPacket, &len, &from)) {
        _buf[len] = 0; // zero out remaining string

        #if defined(RFRX_DEBUG_PRINT)
        Serial.println();
        Serial.print("Rec packet from #"); Serial.print(from);
        Serial.print(" [RSSI :"); Serial.print(_rf69.lastRssi()); Serial.print("] : ");

        Serial.println();
        Serial.println(F("REC DATA STRUCTURE:"));
        _printStateData(&_currState);
        #endif

        #if defined(RFRX_DEBUG_SPEED)
        _DEBUG_PlotSpeed();
        #else
        _serialLogData(&_currState);
        #endif

        // end a reply back to the originator client
        /*
        uint8_t ackMsg[] = "Data Rec.";
        if (!_rf69Manager.sendtoWait(ackMsg, sizeof(ackMsg), from)){
          #if defined(RFRX_DEBUG_PRINT)
          Serial.println(F("Send failed (no ack)"));
          #endif
        }
        */
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;
  bool _start_flag = true;

  // Declare instance of the packet to send
  dataPacket_t _currState;

  // Radio class and radio ackMsg manager class
  // RH_RF69_MAX_MESSAGE_LEN = 60
  RH_RF69 _rf69 = RH_RF69(RFM69_CS, RFM69_INT);
  RHReliableDatagram _rf69Manager = RHReliableDatagram(_rf69, CLIN_RF_ADDR);
  int16_t _packetnum = 0;  // packet counter, we increment per xmission

  // Buffer and acknowledgement message
  uint8_t _buf[RH_RF69_MAX_MESSAGE_LEN];

  // Time Stamp Diff
  uint32_t _lastTime = 0;

  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  #if defined(RFRX_DEBUG_SPEED)
  void _DEBUG_PlotSpeed(){
    Serial.print(_currState.state.onTime-_lastTime); Serial.print(",");
    _lastTime = _currState.state.onTime;
    Serial.print(_currState.state.wheelSpeedL); Serial.print(",");
    Serial.print(_currState.state.wheelSpeedR); Serial.print(",");
    Serial.println();
  }
  #endif
};
#endif
