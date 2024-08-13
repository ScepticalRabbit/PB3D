//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// TEST: wheel encoders
//---------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
*/

#include <Arduino.h>
#include "Timer.h"
#include "Encoder.h"

// Digital pins for left and right encoders
// NOTE: pins 5 and 7 cannot be interrupt at same time on Xiao
static int encPinAL = 2, encPinBL = 3;
static int encPinAR = 4, encPinBR = 5;

// Declare encoder objects in main, pass pointers to move object
// NOTE: encoders must be in main to attach interrupts
Encoder encoderL = Encoder(encPinAL,encPinBL);
Encoder encoderR = Encoder(encPinAR,encPinBR);

// Interrupt function prototypes
void updateEncLA();
void updateEncLB();
void updateEncRA();
void updateEncRB();

//---------------------------------------------------------------------------
// SETUP
void setup(){
    // Start the serial
    Serial.begin(115200);
    // Only use below to stop start up until UDB cable connected
    while(!Serial){}
    Serial.println("TEST ENCODER");

    // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
    attachInterrupt(digitalPinToInterrupt(encPinAL),
                updateEncLA,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPinAR),
                updateEncRA,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPinBL),
                updateEncLB,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encPinBR),
                updateEncRB,CHANGE);
}

//---------------------------------------------------------------------------
// LOOP
void loop(){

}

//---------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
void updateEncLA(){
  encoderL.update_not_equal();
  printEncCount(&encoderL);
}
void updateEncLB(){
  encoderL.update_equal();
  printEncCount(&encoderL);
}
void updateEncRA(){
  encoderR.update_equal();
  printEncCount(&encoderR);
}
void updateEncRB(){
  encoderR.update_not_equal();
  printEncCount(&encoderR);
}

void printEncCount(Encoder* inEnc){
    Serial.println(inEnc->get_count());
}

