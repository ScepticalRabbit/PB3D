#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/PID.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
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

PID::PID(bool inCmdOn, double kp, double ki, double kd){
    _command_on = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

PID::PID(bool inCmdOn, double kp, double ki, double kd, uint16_t sampTime){
    _command_on = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sample_time_ms = sampTime;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void PID::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP

// This version directly returns the output, useful for position control
void PID::update(double input){
    // If the PID is turned off do nothing
    if(!_auto_on){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pid_timer.finished()){
        _output = _compute_PID(input);
        // Start the timer again
        _pid_timer.start(_sample_time_ms);
    }
}

// This version of update adds the PID output to the command value, for velocity control
void PID::update(double inCommand, double input){
    // If the PID is turned off do nothing
    if(!_auto_on){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pid_timer.finished()){

        double outPID = _compute_PID(input);
        outPID = inCommand + outPID;
        _output = constrain(outPID,_out_min,_out_max);

        // Start the timer again
        _pid_timer.start(_sample_time_ms);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void PID::set_output(double output){
    _output = output;

    // Reset the current output based on the new limits
    _output = constrain(_output,_out_min,_out_max);
    // Reset the integral term based on the new output limits
    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_PID_gains(double kp, double ki, double kd){
    if(kp<0 || ki<0 || kd<0){return;}

    double sampleTimeInSec = ((double)_sample_time_ms)/1000;
    _kp = kp;
    _ki = ki * sampleTimeInSec;
    _kd = kd / sampleTimeInSec;

    if(_controller_dir == PID_REVERSE){
        _kp = (0.0 - _kp);
        _ki = (0.0 - _ki);
        _kd = (0.0 - _kd);
    }
}

void PID::set_Pgain_only(double kp){
    if(kp<0){return;}

    _kp = kp;

    if(_controller_dir == PID_REVERSE){
        _kp = (0.0 - _kp);
    }
}

void PID::set_sample_time(int newSampleTime){
    if (newSampleTime > 0){
        double ratio  = (double)newSampleTime
                        / (double)_sample_time_ms;
        _ki *= ratio;
        _kd /= ratio;
        _sample_time_ms = (unsigned long)newSampleTime;
    }
}

void PID::set_output_limits(double outMin, double outMax){
    // Error check that the limits are the correct way round
    if(outMin > outMax){return;}

    // Store the new limits
    _out_min = outMin;
    _out_max = outMax;

    // Reset the current output based on the new limits
    _output = constrain(_output,_out_min,_out_max);
    // Reset the integral term based on the new output limits
    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_command_limits(double cmdMin, double cmdMax){
    // Error check that the limits are the correct way round
    if(cmdMin > cmdMax){return;}

    // Store the new limits
    _cmd_min = cmdMin;
    _cmd_max = cmdMax;

    // Reset the current output based on the new limits
    _output = constrain(_output,_out_min,_out_max);
    // Reset the integral term based on the new output limits
    _int_term = _constrain_by_command_mode(_int_term);
}

void PID::set_controller_on(uint8_t inFlag){
    bool onFlag = (inFlag == PID_ON);
    // If we go from manual to auto then initialise PID
    if(onFlag && !_auto_on){
        _intialise();
    }
    _auto_on = onFlag;
}

void PID::set_controller_dir(uint8_t inDir){
    // If we are changing direction flip the sign of the gains
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
    // Reset the integral term based on the new output limits
    _int_term = _constrain_by_command_mode(_int_term);
    if (_command_on){
        _input = 0.0;
        _last_input = 0.0;
        _int_term = 0.0;
        _output = 0.0;
    }
    // Reset the integral term based on the new output limits
    _int_term = _constrain_by_command_mode(_int_term);
}

// main part of the update method used for overloading
double PID::_compute_PID(double input){
    // Compute the error between the input and the set point
    _error = _set_point - input;
    _prop_term = _kp*_error;

    // Calculate the integral term using the cummulative error
    _int_term += (_ki*_error);
    // Clamp the integral term to avoid windup
    _int_term = _constrain_by_command_mode(_int_term);

    // Calculate the difference in input and use this for the derivative term
    double inputDiff = (input - _last_input);
    _deriv_term = _kd*inputDiff;

    // Compute PID output using the previous error terms
    double outPID = _prop_term+_int_term+_deriv_term;
    // Clamp the output to avoid unrealistic values
    outPID = _constrain_by_command_mode(outPID);

    // Save variables for next update and return the command
    _last_input = input;
    return outPID;
}

double PID::_constrain_by_command_mode(double inVal){
    if (_command_on){
        return constrain(inVal,_cmd_min,_cmd_max);
    }
    else{
        return constrain(inVal,_out_min,_out_max);
    }
}