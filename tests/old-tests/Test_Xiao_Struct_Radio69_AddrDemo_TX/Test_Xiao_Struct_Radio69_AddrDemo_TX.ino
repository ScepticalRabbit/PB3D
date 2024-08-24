//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// Test Code - RF TX/RX with data structures
//-----------------------------------------------------------------------------
// Based on Adafruit RF example

// Headers for SPI connection and radio
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include "PB3DTimer.h"

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

// Radio class and radio data manager class
// RH_RF69_MAX_MESSAGE_LEN = 60
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, SERV_RF_ADDR);
int16_t packetnum = 0;  // packet counter, we increment per xmission

// Radio timer
uint16_t radioSendInt = 100; // ms
uint16_t radioTimeOut = 50; // ms
uint32_t radioStart = 0, radioEnd = 0;
Timer radioTimer = Timer();

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
  if (!rf69_manager.init()) {
    Serial.println(F("RF TX: Failed to init RF TX"));
    while(1);
  }
  Serial.println(F("RF TX: initialised."));

  // RF: set parameters
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("RF TX: setFrequency failed");
  }
  rf69.setTxPower(14, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

  // RF: Encryption
  uint8_t key[] = { 0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08,
                    0x04, 0x05, 0x09, 0x08, 0x02, 0x01, 0x03, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RF: RFM69 TX radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  // RF: Start timer
  radioTimer.start(radioSendInt);

  // INIT CLASS:
  currState.state.mood = 1;
  currState.state.task = 2;
  currState.state.collisionFlags[0] = true;
  currState.state.collisionFlags[1] = false;
  currState.state.collisionFlags[2] = true;
  currState.state.collisionFlags[3] = false;
  currState.state.wheelSpeed = 202.2;

  Serial.println(F("INITIAL DATA STRUCT"));
  printRFDataStruct();
}

// Dont put this on the stack:
// RH_RF69_MAX_MESSAGE_LEN = 60
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

//----------------------------------------------------------------------------
// MAIN LOOP
void loop() {
  if(radioTimer.finished()){
    radioTimer.start(radioSendInt);
    radioStart = millis();

    // Print the data structure to be sent
    Serial.println(F("SENDING DATA STRUCTURE:"));
    printRFDataStruct();

    // Send data structure to the destination as a byte array
    if (rf69_manager.sendtoWait(currState.rfPacket,PACKET_SIZE,DEST_RF_ADDR)) {
      uint8_t len = sizeof(buf);
      uint8_t from;

      if (rf69_manager.recvfromAckTimeout(buf, &len, radioTimeOut, &from)) {
        buf[len] = 0; // zero out remaining string

        Serial.print("Reply from #"); Serial.print(from);
        Serial.print(" [RSSI :"); Serial.print(rf69.lastRssi()); Serial.print("] : ");
        Serial.println((char*)buf);

      } else {
        Serial.println(F("No reply..."));
      }
    } else {
      Serial.println(F("Send failed (no ack)."));
    }

    radioEnd = millis();
    Serial.println();
    Serial.print("Radio time = ");
    Serial.print(radioEnd-radioStart);
    Serial.println("ms");
    Serial.println();
  }
}
//----------------------------------------------------------------------------

void printRFDataStruct(){
  Serial.print(F("Mood: "));
  Serial.print(currState.state.mood);
  Serial.print(F("; "));

  Serial.print(F("TaskManager: "));
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
