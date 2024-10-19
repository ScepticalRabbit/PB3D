//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "IMUSensor.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void IMUSensor::begin(){
    if (!_cal.begin()) {
        Serial.println(F("IMU: Failed to init calib helper."));
    }
    else if(!_cal.loadCalibration()){
        Serial.println(F("IMU: No calibration loaded/found."));
    }
    else{
        Serial.println(F("IMU: Calibration helper initialised."));
    }

    if (!_init_IMU()) {
        Serial.println(F("IMU: Failed to find NXP sensors"));
        _enabled = false;
        //while(true){delay(10);}
    }
    else{
        Serial.println(F("IMU: NXP IMU initialised."));
    }

    _filter.begin(_IMU_filter_freq);
    _IMU_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void IMUSensor::update(){
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

bool IMUSensor::_init_IMU() {
    if (!_fxos_accel_mag.begin() || !_fxas_gyro.begin()) {
        return false;
    }
    _accel = _fxos_accel_mag.getAccelerometerSensor();
    _gyro = &_fxas_gyro;
    _mag = _fxos_accel_mag.getMagnetometerSensor();

    return true;
}