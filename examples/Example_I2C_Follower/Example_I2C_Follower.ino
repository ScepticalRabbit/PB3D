/*
  Made on 8 June 2021
  By Amirmohammad Shojaei
  Home
*/

#include <Wire.h>

int x;
int value=0; 

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);    // start serial for output

  pinMode(9,OUTPUT);
}

void loop() {
  delay(100);
  analogWrite(9,x);

  int t=analogRead(A0);          //Read from A0 & Save to value
  value=map(t,0,1023,0,255);
}

// function that executes whenever data is received from master
void receiveEvent() {
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read();       // receive byte as a character
    Serial.print(c);         // print the character
  }
   x = Wire.read();    // receive byte as an integer
   Serial.println(x);         // print the integer
}

// function that executes whenever data is requested by master
void requestEvent() {
  Wire.write(value); // respond with message of 1 byte
  // as expected by master
}
