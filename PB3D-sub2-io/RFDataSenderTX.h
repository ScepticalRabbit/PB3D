//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef RFDATASENDERTX_H
#define RFDATASENDERTX_H

//----------------------------------------------------------------------------
// INCLUDES
// Include required classes for SPI and RF chip
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include "Timer.h"

#include "StateData.h"

//----------------------------------------------------------------------------
// DEFINITIONS
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.2

// SPI Radio Pinouts for the Xiao
#define RFM69_CS      7
#define RFM69_INT     3
#define RFM69_RST     6
#define LED           13

// Server and destination address - this board is the server
#define DEST_RF_ADDR   1
#define SERV_RF_ADDR   2

//----------------------------------------------------------------------------
// CLASS
class RFDataSenderTX{
public:
  RFDataSenderTX(){
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // RF: reset pin
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    Serial.println(F("RF: TX Radio Send Struct"));
    Serial.println();

    // RF: Reset the RF chip
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    // RF: initialise chip
    if (!_rf69_manager.init()) {
      Serial.println(F("RF TX: Failed to init RF TX"));
      _enabled = false;
      //while(1);
    }
    else{
      Serial.println(F("RF TX: initialised."));
      _rf69_manager.setTimeout(_sendTimeout);
      _rf69_manager.setRetries(_sendRetries);

      // RF: set parameters
      if (!_rf69.setFrequency(RF69_FREQ)) {
        Serial.println("RF TX: setFrequency failed");
      }
      _rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

      // RF: Encryption
      /*
      uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                        0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
      _rf69.setEncryptionKey(key);
      */

      Serial.print("RF: RFM69 TX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

      // RF: Start timer
      _radioTimer.start(_radioSendInt);

      Serial.println(F("INITIAL DATA STRUCT"));
      _init_state_data(&_curr_state);
      _print_state_data(&_curr_state);

      // Send the struct and see if there is a listener
      if(!_sendStateStruct()){
        Serial.println(F("RF TX: initial data send failed!"));
        //_enabled = false;
      }
    }
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    // RADIO
    /*
    if(_radioTimer.finished()){
      _radioTimer.start(_radioSendInt);
      _sendStateStruct();
    }
    */
    if(_newPacket){
      _newPacket = false;
      _sendStateStruct();
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}
  void setNewPacket(bool inFlag){_newPacket = inFlag;}

  void setStateByte(byte inByte, int16_t index){
    _curr_state.dataPacket[index] = inByte;
  }

  //---------------------------------------------------------------------------
  // DIAGNOSTICS
  void printStateData(){
    _print_state_data(&_curr_state);
  }

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  bool _sendStateStruct(){
    bool sendStatus = false;

    // Print the data structure to be sent
    //Serial.println(F("SENDING DATA STRUCTURE:"));
    //_print_state_data(&_curr_state);

    _radioStart = millis();

    // Send data structure to the destination as a byte array
    if (_rf69_manager.sendtoWait(_curr_state.dataPacket,PACKET_SIZE,DEST_RF_ADDR)) {
      /*
      uint8_t len = sizeof(_buf);
      uint8_t from;

      if (_rf69_manager.recvfromAckTimeout(_buf, &len, _radioTimeOut, &from)) {
        _buf[len] = 0; // zero out remaining string

        //Serial.print("Reply from #"); Serial.print(from);
        //Serial.print(" [RSSI :"); Serial.print(_rf69.lastRssi()); Serial.print("] : ");
        //Serial.println((char*)_buf);

        sendStatus = true;
      } else {
        Serial.println(F("No reply..."));
        sendStatus = false;
      }
      */
    }
    else{
      Serial.println(F("RF TX: send failed, no ack."));
      sendStatus = false;
    }

    _radioEnd = millis();
    //Serial.println();
    //Serial.print("Radio time = ");
    Serial.print(_radioEnd-_radioStart);
    //Serial.println("ms");
    Serial.println();

    return sendStatus;
  }

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;
  bool _start_flag = true;

  // DATA PACKET - I2C and Radio
  bool _newPacket = false;
  dataPacket_t _curr_state;

  // RADIO VARIABLES
  // Radio class and radio data manager class
  // RH_RF69_MAX_MESSAGE_LEN = 60
  RH_RF69 _rf69 = RH_RF69(RFM69_CS, RFM69_INT);
  RHReliableDatagram _rf69_manager = RHReliableDatagram(_rf69, SERV_RF_ADDR);
  int16_t _packetnum = 0;  // packet counter

  uint8_t _sendRetries = 2;
  uint16_t _sendTimeout = 13; //ms

  // Radio timer
  //uint16_t _radioSendInt = 200; // ms
  uint16_t _radioSendInt = STATEDATA_UPD_TIME; // ms
  uint16_t _radioAckTimeOut = STATEDATA_UPD_TIME; // ms
  uint32_t _radioStart = 0, _radioEnd = 0;
  Timer _radioTimer = Timer();

  // Buffer:
  uint8_t _buf[RH_RF69_MAX_MESSAGE_LEN];
};
#endif // RFDATASENDERTX_H
