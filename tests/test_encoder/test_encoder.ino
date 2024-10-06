//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// TEST: wheel encoders
//---------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
*/

#include <Arduino.h>
#include "PB3DTimer.h"
#include "Encoder.h"

// Digital pins for left and right encoders
// NOTE: pins 5 and 7 cannot be interrupt at same time on Xiao
static int encoder_pinA_left = 2, encoder_pinB_left = 3;
static int encoder_pinA_right = 4, encoder_pinB_right = 5;

// Declare encoder objects in main, pass pointers to move object
// NOTE: encoders must be in main to attach interrupts
Encoder encoder_left = Encoder(encoder_pinA_left,encoder_pinB_left);
Encoder encoder_right = Encoder(encoder_pinA_right,encoder_pinB_right);

// Interrupt function prototypes
void update_encoder_left_pina();
void update_encoder_left_pinb();
void update_encoder_right_pina();
void update_encoder_right_pinb();

//---------------------------------------------------------------------------
// SETUP
void setup(){
    // Start the serial
    Serial.begin(115200);
    // Only use below to stop start up until UDB cable connected
    while(!Serial){}
    Serial.println("TEST ENCODER");

    // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
    attachInterrupt(digitalPinToInterrupt(encoder_pinA_left),
                update_encoder_left_pina,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_pinA_right),
                update_encoder_right_pina,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_pinB_left),
                update_encoder_left_pinb,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_pinB_right),
                update_encoder_right_pinb,CHANGE);
}

//---------------------------------------------------------------------------
// LOOP
void loop(){

}

//---------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
void update_encoder_left_pina(){
  encoder_left.update_not_equal();
  printEncCount(&encoder_left);
}
void update_encoder_left_pinb(){
  encoder_left.update_equal();
  printEncCount(&encoder_left);
}
void update_encoder_right_pina(){
  encoder_right.update_equal();
  printEncCount(&encoder_right);
}
void update_encoder_right_pinb(){
  encoder_right.update_not_equal();
  printEncCount(&encoder_right);
}

void printEncCount(Encoder* inEnc){
    Serial.println(inEnc->get_count());
}

