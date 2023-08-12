//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================


        // data to be sent
struct I2cTxStruct {
    char textA[16];         // 16 bytes
    int valA;               //  2
    unsigned long valB;     //  4
    byte padding[10];       // 10
                            //------
                            // 32
};

I2cTxStruct txData = {"xxx", 236, 0};

bool newTxData = false;


        // I2C control stuff
#include <Wire.h>

const byte thisAddress = 8; // these need to be swapped for the other Arduino
const byte otherAddress = 9;


        // timing variables
unsigned long prevUpdateTime = 0;
unsigned long updateInterval = 500;

//=================================

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting I2C Master demo\n");

        // set up I2C
    Wire.begin(thisAddress); // join i2c bus

}

//============

void loop() {

        // this function updates the data in txData
    updateDataToSend();
        // this function sends the data if one is ready to be sent
    transmitData();
}

//============

void updateDataToSend() {

    if (millis() - prevUpdateTime >= updateInterval) {
        prevUpdateTime = millis();
        if (newTxData == false) { // ensure previous message has been sent

            char sText[] = "SendA";
            strcpy(txData.textA, sText);
            txData.valA += 10;
            if (txData.valA > 300) {
                txData.valA = 236;
            }
            txData.valB = millis();
            newTxData = true;
        }
    }
}

//============

void transmitData() {

    if (newTxData == true) {
        Wire.beginTransmission(otherAddress);
        Wire.write((byte*) &txData, sizeof(txData));
        Wire.endTransmission();    // this is what actually sends the data

            // for demo show the data that as been sent
        Serial.print("Sent ");
        Serial.print(txData.textA);
        Serial.print(' ');
        Serial.print(txData.valA);
        Serial.print(' ');
        Serial.println(txData.valB);

        newTxData = false;
    }
}
