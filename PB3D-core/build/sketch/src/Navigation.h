#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Navigation.h"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: NAVIGATION
//---------------------------------------------------------------------------
/*
The ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

#include "Timer.h"
#include "Encoder.h"
#include "IMUSensor.h"

//#define NAV_DEBUG_TIMER
//#define NAV_DEBUG_POS

class Navigation{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Navigation(Encoder* encL, Encoder* encR, IMUSensor* inIMU){
    _encoder_L = encL;
    _encoder_R = encR;
    _IMUObj = inIMU;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    if(!_IMUObj->getEnabledFlag()){
      Serial.println(F("NAV: IMU disabled, navigation disabled."));
    }
    else{
      Serial.println(F("NAV: IMU found, navigation enabled."));
    }

    _navTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_isEnabled){return;}

    if(_navTimer.finished()){
      _navTimer.start(_navUpdateTime);

      // Store the last calculated coords as previous
      _posXPrev = _posXNext;
      _posYPrev = _posYNext;

      // Get the speed of the robots centroid and current heading
      _velC = (_encoder_L->getSmoothSpeedMMPS()+
                    _encoder_R->getSmoothSpeedMMPS()) / 2.0;
      _heading = _IMUObj->getHeadAng();

      // Calculate the velocity components (X points to front face of robot)
      _velX = _velC*cos((_heading/180.0)*float(PI));
      _velY = _velC*sin((_heading/180.0)*float(PI));

      // Update the predicted position
      _posXNext = _posXPrev + _velX*_dt;
      _posYNext = _posYPrev + _velY*_dt;     

      // Debug prints update every X interations based on debugFreq
      if(_debugCount++ >= _debugFreq){
        _debugCount = 0; // Reset the debug count

        #if defined(NAV_DEBUG_TIMER)
          Serial.print(F("NAV: update took ")); 
          Serial.print(_navTimer.getTime());
          Serial.println(F("ms"));
        #endif

        #if defined(NAV_DEBUG_POS)
          Serial.print(F("NAV: Pos X= "));
          Serial.print(_posXNext);
          Serial.print(F("mm, Pos Y= "));
          Serial.print(_posYNext);
          Serial.println(F("mm"));
        #endif
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  float getPosX(){return _posXNext;}
  float getPosY(){return _posYNext;}
  float getVelX(){return _velX;}
  float getVelY(){return _velY;}
  float getVelC(){return _velC;}
  float getHeading(){return _heading;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

  // Debug variables
  uint8_t _debugCount = 0;
  uint8_t _debugFreq = 10;

  // Navigation Objects - Encoders and IMU
  Encoder* _encoder_L;
  Encoder* _encoder_R;
  IMUSensor* _IMUObj;

  // Navigation Variables
  int16_t _navUpdateTime = 20; 
  float _dt = float(_navUpdateTime)/1000.0;
  Timer _navTimer = Timer();

  float _velC = 0.0, _velX = 0.0, _velY = 0.0;
  float _heading = 0.0;
  float _posXNext = 0.0, _posYNext = 0.0;
  float _posXPrev = 0.0, _posYPrev = 0.0;
};
#endif //
