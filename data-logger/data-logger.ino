//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// RF DATA LOGGER RX - Use with Xiao+RF chip to log data to laptop
//-----------------------------------------------------------------------------

#include "RFDataLoggerRX.h"
#include "Timer.h"

RFDataLoggerRX rfLogger;

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  //while(!Serial){delay(10);}
  rfLogger.begin();
  delay(1000);
}

//----------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  rfLogger.update();
}
//----------------------------------------------------------------------------
