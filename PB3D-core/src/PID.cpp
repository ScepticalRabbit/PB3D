//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Based on code found here:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//------------------------------------------------------------------------------

#include "PID.h"

//---------------------------------------------------------------------------
// CONSTRUCTORS
PID::PID(bool inCmdOn){
    _command_on = inCmdOn;
}

PID::PID(bool inCmdOn, float kp, float ki, float kd){
    _command_on = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

PID::PID(bool inCmdOn, float kp, float ki, float kd, uint16_t sampTime){
    _command_on = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sample_time_ms = sampTime;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void PID::begin(){
    _pid_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP

void PID::update(float input){
    if(!_auto_on){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pid_timer.finished()){
        _pid_timer.start(_sample_time_ms);
        _output = _compute_PID(input);
    }
}

void PID::update(float command, float input){
    // If the PID is turned off do nothing
    if(!_auto_on){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pid_timer.finished()){
        _pid_timer.start(_sample_time_ms);

        float out_PID = _compute_PID(input);
        out_PID = command + out_PID;
        _output = constrain(out_PID,_out_min,_out_max);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void PID::set_output(float output){
    _output = output;

    _output = constrain(_output,_out_min,_out_max);
    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_PID_gains(float kp, float ki, float kd){
    if(kp<0 || ki<0 || kd<0){return;}

    float sampleTimeInSec = ((float)_sample_time_ms)/1000;
    _kp = kp;
    _ki = ki * sampleTimeInSec;
    _kd = kd / sampleTimeInSec;

    if(_controller_dir == PID_REVERSE){
        _kp = (0.0 - _kp);
        _ki = (0.0 - _ki);
        _kd = (0.0 - _kd);
    }
}

void PID::set_Pgain_only(float kp){
    if(kp<0){return;}

    _kp = kp;

    if(_controller_dir == PID_REVERSE){
        _kp = (0.0 - _kp);
    }
}

void PID::set_sample_time(int newSampleTime){
    if (newSampleTime > 0){
        float ratio  = (float)newSampleTime
                        / (float)_sample_time_ms;
        _ki *= ratio;
        _kd /= ratio;
        _sample_time_ms = (unsigned long)newSampleTime;
    }
}

void PID::set_output_limits(float outMin, float outMax){
    if(outMin > outMax){return;}

    _out_min = outMin;
    _out_max = outMax;

    _output = constrain(_output,_out_min,_out_max);

    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_command_limits(float cmdMin, float cmdMax){
    if(cmdMin > cmdMax){return;}

    _cmd_min = cmdMin;
    _cmd_max = cmdMax;

    _output = constrain(_output,_out_min,_out_max);

    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_controller_on(uint8_t inFlag){
    bool onFlag = (inFlag == PID_ON);

    if(onFlag && !_auto_on){
        _intialise();
    }
    _auto_on = onFlag;
}

void PID::set_controller_dir(uint8_t inDir){
    if(inDir != _controller_dir){
        _kp = (0.0 - _kp);
        _ki = (0.0 - _ki);
        _kd = (0.0 - _kd);
    }
    _controller_dir = inDir;
}

//---------------------------------------------------------------------------
// Private Functions

// Used when turning the PID on
void PID::_intialise(){
    _last_input = _input;
    _int_term = _output;

    _int_term = _constrain_by_command_mode(_int_term);
    if (_command_on){
        _input = 0.0;
        _last_input = 0.0;
        _int_term = 0.0;
        _output = 0.0;
    }

    _int_term = _constrain_by_command_mode(_int_term);
}

float PID::_compute_PID(float input){

    _error = _set_point - input;
    _prop_term = _kp*_error;


    _int_term += (_ki*_error);
    // Clamp the integral term to avoid windup
    _int_term = _constrain_by_command_mode(_int_term);

     float inputDiff = (input - _last_input);
    _deriv_term = _kd*inputDiff;


    float out_PID = _prop_term+_int_term+_deriv_term;
    out_PID = _constrain_by_command_mode(out_PID);

    // Save variables for next update and return the command
    _last_input = input;
    return out_PID;
}

float PID::_constrain_by_command_mode(float inVal){
    if (_command_on){
        return constrain(inVal,_cmd_min,_cmd_max);
    }
    else{
        return constrain(inVal,_out_min,_out_max);
    }
}