//---------------------------------------------------------------------------
// PET BOT - PB3D!
// CLASS - IMU
//---------------------------------------------------------------------------
/*
Author: Lloyd Fletcher
Date Created: 22nd December 2022
Date Edited:  22nd December 2022
*/

#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

#include "Timer.h"

//#define IMU_DEBUG_TIMER
//#define IMU_DEBUG_RAW_OUTPUT
//#define IMU_DEBUG_ANGLES

class IMU{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR
  IMU(){
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    if (!_cal.begin()) {
      Serial.println(F("IMU: Failed to init calib helper"));
    } else if (!_cal.loadCalibration()) {
      Serial.println(F("IMU: No calibration loaded/found"));
    }

    if (!_init_IMU()) {
      Serial.println(F("IMU: Failed to find NXP sensors"));
      _enabled = false;
      //while(true){delay(10);}
    }

    _filter.begin(_IMU_filter_freq);
    _IMU_timer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

    if(_IMU_timer.finished()){
      _IMU_timer.start(_IMU_update_time);

      // Read all the motion sensors
      // Driven by I2C read time which is driven by I2C clock
      // Test on M4 = 4ms with default clock and 1ms with 400kHz clock
      // sensors_event_t aEvent, gEvent, mEvent;
      _accel->getEvent(&_accel_event);
      _gyro->getEvent(&_gyro_event);
      _mag->getEvent(&_mag_event);

      // Apply the calibration to the sensor values
      _cal.calibrate(_mag_event);
      _cal.calibrate(_accel_event);
      _cal.calibrate(_gyro_event);

      // Gyroscope needs to be converted from Rad/s to Degree/s
      // the rest are not unit-important
      float gx = _gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
      float gy = _gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
      float gz = _gyro_event.gyro.z * SENSORS_RADS_TO_DPS;

      // Update the SensorFusion filter
      // Supposedly computationally intensive - test on M4 = 1ms or less
      _filter.update(gx, gy, gz,
                    _accel_event.acceleration.x, _accel_event.acceleration.y, _accel_event.acceleration.z,
                    _mag_event.magnetic.x, _mag_event.magnetic.y, _mag_event.magnetic.z);

      // Get the Euler angles from the filter
      _roll = _filter.getRoll();
      _pitch = _filter.getPitch();
      _heading = _filter.getYaw();

      // Debug prints update every X interations based on debugFreq
      if(_debug_count++ >= _debug_freq){
        _debug_count = 0; // Reset the debug count

        #if defined(IMU_DEBUG_TIMER)
          Serial.print("IMU: update took ");
          Serial.print(_IMU_timer.get_time());
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
  bool get_enabled_flag(){return _enabled;}

  float get_roll_angle(){return _roll;}
  float get_pitch_angle(){return _pitch;}
  float get_head_angle(){return _heading;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  bool _init_IMU(void) {
    if (!_fxos_accel_mag.begin() || !_fxas_gyro.begin()) {
      return false;
    }
    _accel = _fxos_accel_mag.getAccelerometerSensor();
    _gyro = &_fxas_gyro;
    _mag = _fxos_accel_mag.getMagnetometerSensor();

    return true;
  }

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  // Debug variables
  uint8_t _debug_count = 0;
  uint8_t _debug_freq = 10;

  // Timers
  float _IMU_filter_freq = 100.0; // Hz
  int16_t _IMU_update_time = int16_t(1000.0/_IMU_filter_freq); // ms
  Timer _IMU_timer = Timer();

  // Euler angle variables
  float _heading = 0.0, _roll = 0.0, _pitch = 0.0;

  // Objects for NXP IMU
  Adafruit_FXOS8700 _fxos_accel_mag = Adafruit_FXOS8700(0x8700A, 0x8700B);
  Adafruit_FXAS21002C _fxas_gyro = Adafruit_FXAS21002C(0x0021002C);

  // Pointers to the sensor objects
  Adafruit_Sensor *_accel, *_gyro, *_mag;

  // pick your filter! slower == better quality output
  Adafruit_NXPSensorFusion _filter; // slowest
  //Adafruit_Madgwick _filter;  // faster than NXP
  //Adafruit_Mahony _filter;  // fastest/smalleset

  // Need to change this for EEPROM but M0 and M4's use this
  Adafruit_Sensor_Calibration_SDFat _cal;

  // Event classes for holding sensor readings
  sensors_event_t _accel_event, _gyro_event, _mag_event;
};
#endif //
