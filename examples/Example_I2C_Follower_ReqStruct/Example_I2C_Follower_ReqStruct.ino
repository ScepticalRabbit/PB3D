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
bool rqSent = false;


        // I2C control stuff
#include <Wire.h>

const byte thisAddress = 9; // these need to be swapped for the other Arduino
const byte otherAddress = 8;



//=================================

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting I2C SlaveRespond demo\n");

        // set up I2C
    Wire.begin(thisAddress); // join i2c bus
    Wire.onReceive(receiveEvent); // register function to be called when a message arrives
    Wire.onRequest(requestEvent); // register function to be called when a request arrives

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

}

//============

void updateDataToSend() {

        // update the data after the previous message has been
        //    sent in response to the request
        // this ensures the new data will ready when the next request arrives
    if (rqSent == true) {
        rqSent = false;

        char sText[] = "SendB";
        strcpy(txData.textA, sText);
        txData.valA += 10;
        if (txData.valA > 300) {
            txData.valA = 236;
        }
        txData.valB = millis();

    }
}

//=========

void showTxData() {

            // for demo show the data that as been sent
        Serial.print("Sent ");
        Serial.print(txData.textA);
        Serial.print(' ');
        Serial.print(txData.valA);
        Serial.print(' ');
        Serial.println(txData.valB);

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

//============

        // this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {

    if (newRxData == false) {
            // copy the data to rxData
        Wire.readBytes( (byte*) &rxData, numBytesReceived);
        newRxData = true;
    }
    else {
            // dump the data
        while(Wire.available() > 0) {
            byte c = Wire.read();
        }
    }
}

//===========

void requestEvent() {
    Wire.write((byte*) &txData, sizeof(txData));
    rqSent = true;
}
