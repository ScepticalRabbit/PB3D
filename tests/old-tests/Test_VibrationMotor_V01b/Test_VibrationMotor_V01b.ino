int MoPin = 8;    // vibrator Grove connected to digital pin 8
 
void setup()  {
    pinMode( MoPin, OUTPUT );
}
 
void loop()  {
 
    digitalWrite(MoPin, HIGH);
    delay(1000);
 
    digitalWrite(MoPin, LOW);
    delay(1000);
}
