#include <Wire.h>
#include "RFDataSenderTX.h"
#include "Timer.h"

// Address for nervous system peripherial sensor array
#define NERVSYS_ADDR 9

// This class owns the data packet
RFDataSenderTX sender;

// Timer for printing the data packet for diagnostics
Timer _printTimer = Timer();
uint16_t _printTime = 100; // ms

//----------------------------------------------------------------------------
// I2C HANDLER FUNCTIONS
void requestEvent(){

}

void receiveEvent(int bytesRec){
  int16_t ii = 0;
  while(0<Wire.available()){
    if(ii < PACKET_SIZE){
      //_curr_state.dataPacket[ii] = Wire.read();
      sender.setStateByte(Wire.read(),ii);
    }
    else{
      Wire.read();
    }
    ii++;
  }
}

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  delay(1000);
  sender.begin();
  delay(1000);

  Wire.begin(NERVSYS_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}

void loop(){
  sender.update();

  if(_printTimer.finished()){
    _printTimer.start(_printTime);

    Serial.println(F("I2C DATA STRUCT REC:"));
    sender.printDataStruct();
  }
}
