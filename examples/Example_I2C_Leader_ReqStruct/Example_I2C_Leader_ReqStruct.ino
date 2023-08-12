//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================


        // data to be sent and received
struct I2cTxStruct {
    char textA[16];         // 16 bytes
    int valA;               //  2
    unsigned long valB;     //  4
    byte padding[10];       // 10
                            //------
                            // 32
};

struct I2cRxStruct {
    char textB[16];         // 16 bytes
    int valC;               //  2
    unsigned long valD;     //  4
    byte padding[10];       // 10
                            //------
                            // 32
};

I2cTxStruct txData = {"xxx", 236, 0};
I2cRxStruct rxData;

bool newTxData = false;
bool newRxData = false;
bool rqData = false;


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
    Serial.println("\nStarting I2C MasterRequest demo\n");

        // set up I2C
    Wire.begin(thisAddress); // join i2c bus
    //~ Wire.onReceive(receiveEvent); // register function to be called when a message arrives

}

//============

void loop() {

        // this bit checks if a message has been received
    if (newRxData == true) {
        showNewData();
        newRxData = false;
    }


        // this function updates the data in txData
    updateDataToSend();
        // this function sends the data if one is ready to be sent
    transmitData();
    requestData();
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
        rqData = true;
    }
}

//=============

void requestData() {
    if (rqData == true) { // just one request following every Tx
        byte stop = true;
        byte numBytes = 32;
        Wire.requestFrom(otherAddress, numBytes, stop);
            // the request is immediately followed by the read for the response
        Wire.readBytes( (byte*) &rxData, numBytes);
        newRxData = true;
        rqData = false;
    }
}

//=============

void showNewData() {

    Serial.print("This just in    ");
    Serial.print(rxData.textB);
    Serial.print(' ');
    Serial.print(rxData.valC);
    Serial.print(' ');
    Serial.println(rxData.valD);
}
