// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include "PB3DTimer.h"
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

// SPI Radio Pinouts for the Xiao
#define RFM69_CS      7
#define RFM69_INT     3
#define RFM69_RST     6
#define LED           13

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define THIS_ADDRESS   2

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, THIS_ADDRESS);

// RH_RF69_MAX_MESSAGE_LEN = 60
int16_t packetnum = 0;  // packet counter, we increment per xmission

// Radio timer
uint16_t radioSendInt = 100; // ms
uint16_t radioTimeOut = 50; // ms
uint32_t radioStart = 0, radioEnd = 0;
Timer radioTimer = Timer();

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  radioTimer.start(radioSendInt);
}

// Dont put this on the stack:
// RH_RF69_MAX_MESSAGE_LEN = 60
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  if(radioTimer.finished()){
    radioTimer.start(radioSendInt);
    radioStart = millis();

    //char radiopacket[20] = "Hello World #";
    // 60 byte test
    char radiopacket[60] = "This is a test.This is a test.This is a test.This is a test";
    itoa(packetnum++, radiopacket+13, 10);
    Serial.print("Sending "); Serial.println(radiopacket);

    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout(buf, &len, radioTimeOut, &from)) {
        buf[len] = 0; // zero out remaining string

        Serial.print("Got reply from #"); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.print("] : ");
        Serial.println((char*)buf);

      } else {
        Serial.println("No reply, is anyone listening?");
      }
    } else {
      Serial.println("Sending failed (no ack)");
    }

    radioEnd = millis();
    Serial.println();
    Serial.print("Radio time = ");
    Serial.print(radioEnd-radioStart);
    Serial.println("ms");
    Serial.println();
  }
}
