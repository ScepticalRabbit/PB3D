#include <Arduino.h>
#include "wiring_private.h"

Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

// the setup routine runs once when you press reset:
void setup() {

  // initialize serial communication at 9600 bits per second:

  Serial.begin(9600);

  mySerial.begin(9600);

  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1

  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
}

// the loop routine runs over and over again forever:
void loop() {

  // read the input on analog pin 0:

  int sensorValue = analogRead(A0);

  // print out the value you read on mySerial wired in loopback:

  mySerial.write(sensorValue);
  delay(10);

  while (mySerial.available()) {

    Serial.print(mySerial.read());

  }

  Serial.println();

  delay(10);        // delay in between reads for stability
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{

  mySerial.IrqHandler();
}
