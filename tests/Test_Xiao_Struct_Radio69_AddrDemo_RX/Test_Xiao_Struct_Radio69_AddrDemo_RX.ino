//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// Test Code - RF TX/RX with ackMsg structures
//-----------------------------------------------------------------------------
// Based on Adafruit RF example

// Headers for SPI connection and radio
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include "Timer.h"

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.2

// SPI Radio Pinouts for the Xiao
#define RFM69_CS      7
#define RFM69_INT     3
#define RFM69_RST     6
#define LED           13

// This board is the client sending ackMsg to the server
#define CLIN_RF_ADDR     1

// Radio class and radio ackMsg manager class
// RH_RF69_MAX_MESSAGE_LEN = 60 
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, CLIN_RF_ADDR);
int16_t packetnum = 0;  // packet counter, we increment per xmission

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

// Declare instance of the packet to send
radioPacket_t currState;

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  
  // RF: reset pin 
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println(F("RF: RX Radio Send Struct"));
  Serial.println();

  // RF: Reset the RF chip
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println(F("RF RX: Failed to init RF RX"));
    while (1);
  }
  Serial.println(F("RF TX: initialised."));
  
  // RF: set parameters
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // RF: Encryption
  uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                    0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 RX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // INIT CLASS:
  currState.state.mood = 0;
  currState.state.task = 0;
  currState.state.collisionFlags[0] = false;
  currState.state.collisionFlags[1] = false;
  currState.state.collisionFlags[2] = false;
  currState.state.collisionFlags[3] = false;
  currState.state.wheelSpeed = 0.0;
  
  Serial.println(F("INITIAL ackMsg STRUCT"));
  printRFMsgStruct();
}

// Dont put this on the stack:
// RH_RF69_MAX_MESSAGE_LEN = 60 
uint8_t ackMsg[] = "Struct Rec.";
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//----------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  if(rf69_manager.available()){
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    if (rf69_manager.recvfromAck(currState.rfPacket, &len, &from)) {
      buf[len] = 0; // zero out remaining string

      Serial.println();
      Serial.print("Rec packet from #"); Serial.print(from);
      Serial.print(" [RSSI :"); Serial.print(rf69.lastRssi()); Serial.print("] : ");

      Serial.println();
      Serial.println(F("REC DATA STRUCTURE:"));
      printRFMsgStruct();

      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(ackMsg, sizeof(ackMsg), from)){
        Serial.println(F("Send failed (no ack)"));
      }
    }
  }
}
//----------------------------------------------------------------------------

void printRFMsgStruct(){
  Serial.print(F("Mood: "));
  Serial.print(currState.state.mood);
  Serial.print(F("; "));
  
  Serial.print(F("Task: "));
  Serial.print(currState.state.task);
  Serial.print(F("; "));

  Serial.print(F("Col Flags: "));
  for(uint8_t ii = 0; ii < 4; ii++){
    if(currState.state.collisionFlags[ii]){
      Serial.print(F("1"));
    }
    else{
      Serial.print(F("0"));
    }
  }
  Serial.print(F("; "));

  Serial.print(F("Speed: "));
  Serial.print(currState.state.wheelSpeed);
  Serial.print(F("; "));
  Serial.println();
}
//----------------------------------------------------------------------------
