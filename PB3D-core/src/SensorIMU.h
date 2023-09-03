//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: SensorIMU
//---------------------------------------------------------------------------
/*
Author: Lloyd Fletcher
*/

#ifndef SENSORIMU_H
#define SENSORIMU_H

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "Timer.h"

//#define IMU_DEBUG_TIMER
//#define IMU_DEBUG_RAW_OUTPUT
//#define IMU_DEBUG_ANGLES

class SensorIMU{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR
  SensorIMU(){
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    if (!_cal.begin()) {
      Serial.println(F("IMU: Failed to init calib helper."));
    } 
    else if(!_cal.loadCalibration()){
      Serial.println(F("IMU: No calibration loaded/found."));
    }
    else{
      Serial.println(F("IMU: Calibration helper initialised."));
    }

    if (!_initIMU()) {
      Serial.println(F("IMU: Failed to find NXP sensors"));
      _isEnabled = false;
      //while(true){delay(10);}
    }
    else{
      Serial.println(F("IMU: NXP IMU initialised."));
    }
 
    _filter.begin(_IMUFilterFreq);
    _IMUTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update(){
    if(!_isEnabled){return;}

    if(_IMUTimer.finished()){
      _IMUTimer.start(_IMUUpdateTime);

      // Read all the motion sensors
      // Driven by I2C read time which is driven by I2C clock
      // Test on M4 = 4ms with default clock and 1ms with 400kHz clock
      // sensors_event_t aEvent, gEvent, mEvent;
      _accel->getEvent(&_aEvent);
      _gyro->getEvent(&_gEvent);
      _mag->getEvent(&_mEvent);

      // Apply the calibration to the sensor values
      _cal.calibrate(_mEvent);
      _cal.calibrate(_aEvent);
      _cal.calibrate(_gEvent);
      
      // Gyroscope needs to be converted from Rad/s to Degree/s
      // the rest are not unit-important
      float gx = _gEvent.gyro.x * SENSORS_RADS_TO_DPS;
      float gy = _gEvent.gyro.y * SENSORS_RADS_TO_DPS;
      float gz = _gEvent.gyro.z * SENSORS_RADS_TO_DPS;
    
      // Update the SensorFusion filter
      // Supposedly computationally intensive - test on M4 = 1ms or less
      _filter.update(gx, gy, gz, 
                    _aEvent.acceleration.x, _aEvent.acceleration.y, _aEvent.acceleration.z, 
                    _mEvent.magnetic.x, _mEvent.magnetic.y, _mEvent.magnetic.z);

      // Get the Euler angles from the filter
      _roll = _filter.getRoll();
      _pitch = _filter.getPitch();
      _heading = _filter.getYaw();

      // Debug prints update every X interations based on debugFreq
      if(_debugCount++ >= _debugFreq){
        _debugCount = 0; // Reset the debug count
        
        #if defined(IMU_DEBUG_TIMER)
          Serial.print("IMU: update took "); 
          Serial.print(_IMUTimer.getTime());
          Serial.println("ms");
        #endif
  
        #if defined(IMU_DEBUG_RAW_OUTPUT)
          Serial.print("IMU: Raw Accel: ");
          Serial.print(accel.acceleration.x, 4); Serial.print(", ");
          Serial.print(accel.acceleration.y, 4); Serial.print(", ");
          Serial.print(accel.acceleration.z, 4); Serial.print(", ");
          Serial.println();
          Serial.print("IMU: Raw Gyro: ");
          Serial.print(gx, 4); Serial.print(", ");
          Serial.print(gy, 4); Serial.print(", ");
          Serial.print(gz, 4); Serial.print(", ");
          Serial.println();
          Serial.print("IMU: Raw Mag: ");
          Serial.print(mag.magnetic.x, 4); Serial.print(", ");
          Serial.print(mag.magnetic.y, 4); Serial.print(", ");
          Serial.print(mag.magnetic.z, 4); Serial.print(",");
          Serial.println();
        #endif
  
        #if defined(IMU_DEBUG_ANGLES)
          Serial.print("IMU: Angles, Head: ");
          Serial.print(_heading);
          Serial.print(", Pitch: ");
          Serial.print(_pitch);
          Serial.print(", Roll: ");
          Serial.println(_roll);
        #endif
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  float getRollAng(){return _roll;}
  float getPitchAng(){return _pitch;}
  float getHeadAng(){return _heading;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  bool _initIMU(void) {
    if (!_fxos.begin() || !_fxas.begin()) {
      return false;
    }
    _accel = _fxos.getAccelerometerSensor();
    _gyro = &_fxas;
    _mag = _fxos.getMagnetometerSensor();
  
    return true;
  }
  
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

  // Debug variables
  uint8_t _debugCount = 0;
  uint8_t _debugFreq = 10;

  // Timers
  float _IMUFilterFreq = 100.0; // Hz
  int16_t _IMUUpdateTime = int16_t(1000.0/_IMUFilterFreq); // ms
  Timer _IMUTimer = Timer();

  // Euler angle variables
  float _heading = 0.0, _roll = 0.0, _pitch = 0.0;

  // Objects for NXP IMU
  Adafruit_FXOS8700 _fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
  Adafruit_FXAS21002C _fxas = Adafruit_FXAS21002C(0x0021002C);

  // Pointers to the sensor objects
  Adafruit_Sensor *_accel, *_gyro, *_mag;

  // pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion _filter; // slowest
  //Adafruit_Madgwick _filter;  // faster than NXP
  //Adafruit_Mahony _filter;  // fastest/smalleset

  // Need to change this for EEPROM but M0 and M4's use this
  Adafruit_Sensor_Calibration_SDFat _cal;

  // Event classes for holding sensor readings
  sensors_event_t _aEvent, _gEvent, _mEvent;
};
#endif //
