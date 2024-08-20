//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS - RFLOGGERTX
//-----------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
Date Created: 10th Dec. 2022
Date Edited:  10th Dec. 2022
*/

#ifndef RFLOGGERTX_H
#define RFLOGGERTX_H

//----------------------------------------------------------------------------
// INCLUDES
// Include required classes for SPI and RF chip
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include "Timer.h"

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
// DATA STRUCTURES
// Declare data structure and union types
typedef struct stateData_t{
  uint8_t mood;
  uint8_t task;
  bool collisionFlags[4];
  float wheelSpeed;
};

typedef union dataPacket_t{
  stateData_t state;
  byte data_packet[sizeof(stateData_t)];
};

#define PACKET_SIZE sizeof(stateData_t)

//----------------------------------------------------------------------------
// CLASS
class RFLoggerTX{
public:
  RFLoggerTX(){
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
      while(1);
    }
    Serial.println(F("RF TX: initialised."));

    // RF: set parameters
    if (!_rf69.setFrequency(RF69_FREQ)) {
      Serial.println("RF TX: setFrequency failed");
    }
    _rf69.setTxPower(14, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

    // RF: Encryption
    uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                      0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
    _rf69.setEncryptionKey(key);

    Serial.print("RF: RFM69 TX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

    // RF: Start timer
    _radioTimer.start(_radioSendInt);

    // INIT CLASS:
    _curr_state.state.mood = 1;
    _curr_state.state.task = 2;
    _curr_state.state.collisionFlags[0] = true;
    _curr_state.state.collisionFlags[1] = false;
    _curr_state.state.collisionFlags[2] = true;
    _curr_state.state.collisionFlags[3] = false;
    _curr_state.state.wheelSpeed = 202.2;

    Serial.println(F("INITIAL DATA STRUCT"));
    printRFDataStruct();
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    if(_radioTimer.finished()){
      _radioTimer.start(_radioSendInt);
      _radioStart = millis();

      // Print the data structure to be sent
      Serial.println(F("SENDING DATA STRUCTURE:"));
      printRFDataStruct();

      // Send data structure to the destination as a byte array
      if (_rf69_manager.sendtoWait(_curr_state.data_packet,PACKET_SIZE,DEST_RF_ADDR)) {
        uint8_t len = sizeof(_buf);
        uint8_t from;

        if (_rf69_manager.recvfromAckTimeout(_buf, &len, _radioTimeOut, &from)) {
          _buf[len] = 0; // zero out remaining string

          Serial.print("Reply from #"); Serial.print(from);
          Serial.print(" [RSSI :"); Serial.print(_rf69.lastRssi()); Serial.print("] : ");
          Serial.println((char*)_buf);

        } else {
          Serial.println(F("No reply..."));
        }
      } else {
        Serial.println(F("Send failed (no ack)."));
      }

      _radioEnd = millis();
      Serial.println();
      Serial.print("Radio time = ");
      Serial.print(_radioEnd-_radioStart);
      Serial.println("ms");
      Serial.println();
    }
  }

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
  void doSomething(){
    if(!_enabled){return;}

    if(_start_flag){
      _start_flag = false;
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
  void printRFDataStruct(){
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
  dataPacket_t _curr_state;

  // Radio class and radio data manager class
  // RH_RF69_MAX_MESSAGE_LEN = 60
  RH_RF69 _rf69 = RH_RF69(RFM69_CS, RFM69_INT);
  RHReliableDatagram _rf69_manager = RHReliableDatagram(_rf69, SERV_RF_ADDR);
  int16_t _packetnum = 0;  // packet counter

  // Radio timer
  uint16_t _radioSendInt = 100; // ms
  uint16_t _radioTimeOut = 50; // ms
  uint32_t _radioStart = 0, _radioEnd = 0;
  Timer _radioTimer = Timer();

  // Buffer:
  uint8_t _buf[RH_RF69_MAX_MESSAGE_LEN];
};
#endif // RFLOGGERTX_H
