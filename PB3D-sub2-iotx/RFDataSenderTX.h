//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
//https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio


#ifndef RFDATASENDERTX_H
#define RFDATASENDERTX_H

// NOTE: cannot break into cpp because PB3DStateData.h will not compile macros
// correctly

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include <PB3DTimer.h>
#include <PB3DStateData.h>

//----------------------------------------------------------------------------
// DEFINITIONS
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.2

// SPI Radio Pinouts for the Xiao
enum ERadioPins{
    RFM69_CS = 7,
    RFM69_INT = 3,
    RFM69_RST = 6,
    LED_PIN = 13,
    RM69_MOSI = 10,
    RM69_MISO = 9,
    RM69_SCK = 8,
};
// Server and destination address - this board is the server
#define DEST_RF_ADDR   1
#define SERV_RF_ADDR   2


class RFDataSenderTX{
public:
  RFDataSenderTX(){}

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
      _rf69_manager.setTimeout(_send_timeout);
      _rf69_manager.setRetries(_send_retries);

      // RF: set parameters
      if (!_rf69.setFrequency(RF69_FREQ)) {
        Serial.println("RF TX: setFrequency failed");
      }
      _rf69.setTxPower(15, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

      // RF: Encryption
      /*
      uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                        0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
      _rf69.setEncryptionKey(key);
      */

      Serial.print("RF: RFM69 TX radio @");
      Serial.print((int)RF69_FREQ);
      Serial.println(" MHz");

      Serial.println(F("INITIAL DATA STRUCT"));
      _init_state_data(&_curr_state);
      _print_state_data(&_curr_state);

      // Send the struct and see if there is a listener
      if(!_send_state_struct()){
        Serial.println(F("RF TX: initial data send failed!"));
      }
    }
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    if(_new_packet){
      _new_packet = false;
      _send_state_struct();
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}
  void set_new_packet(bool inFlag){_new_packet = inFlag;}

  void set_state_byte(byte in_byte, int16_t index){
    _curr_state.data_packet[index] = in_byte;
  }

  //---------------------------------------------------------------------------
  // DIAGNOSTICS
  void printStateData(){
    _print_state_data(&_curr_state);
  }

private:
  bool _send_state_struct(){
    bool send_status = false;

    // Print the data structure to be sent
    //Serial.println(F("SENDING DATA STRUCTURE:"));
    //_print_state_data(&_curr_state);

    _radio_start = millis();

    if (_rf69_manager.sendtoWait(_curr_state.data_packet,PACKET_SIZE,DEST_RF_ADDR)) {
      // REMOVED FOR SPEED
      /*
      uint8_t len = sizeof(_radio_buffer);
      uint8_t from;

      if (_rf69_manager.recvfromAckTimeout(_radio_buffer, &len, _radioTimeOut, &from)) {
        _radio_buffer[len] = 0; // zero out remaining string

        //Serial.print("Reply from #"); Serial.print(from);
        //Serial.print(" [RSSI :"); Serial.print(_rf69.lastRssi()); Serial.print("] : ");
        //Serial.println((char*)_radio_buffer);

        send_status = true;
      } else {
        Serial.println(F("No reply..."));
        send_status = false;
      }
      */
    }
    else{
      Serial.println(F("RF TX: send failed, no ack."));
      send_status = false;
    }

    _radio_end = millis();
    //Serial.println();
    Serial.print("Radio time = ");
    Serial.print(_radio_end-_radio_start);
    Serial.println("ms");
    Serial.println();

    return send_status;
  }

  bool _enabled = true;
  bool _start_flag = true;

  // DATA PACKET - I2C and Radio
  bool _new_packet = false;
  UDataPacket _curr_state;

  // RADIO VARIABLES
  // Radio class and radio data manager class
  // RH_RF69_MAX_MESSAGE_LEN = 60
  RH_RF69 _rf69 = RH_RF69(RFM69_CS, RFM69_INT);
  RHReliableDatagram _rf69_manager = RHReliableDatagram(_rf69, SERV_RF_ADDR);

  //int16_t _packet_num = 0;  // packet counter
  uint8_t _send_retries = 2;
  uint16_t _send_timeout = 13; //ms

  uint32_t _radio_start = 0;
  uint32_t _radio_end = 0;

  uint8_t _radio_buffer[RH_RF69_MAX_MESSAGE_LEN];
};
#endif // RFDATASENDERTX_H
