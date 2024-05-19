#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

Uart Serial2 (&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200);

    // Assign pins 3 & 4 SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);
  

}

uint8_t i=0;
void loop() {
  //Serial.print(i);
  Serial2.write(i++);
  delay(10);
  if (Serial2.available()) {
    Serial.print(" -> 0x"); Serial.print(Serial2.read(), HEX);
    Serial.println();
    delay(10);
  }
  //Serial.println();
  
  delay(10);
}
