#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/PID.cpp"
//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: PID
//---------------------------------------------------------------------------
/*
The PID class is part of the PetBot (PB) program. It used to...

Based on code found here:
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

Author: Lloyd Fletcher
*/
#include "PID.h"

 //---------------------------------------------------------------------------
// CONSTRUCTORS
PID::PID(bool inCmdOn){
    _commandOn = inCmdOn;
}

PID::PID(bool inCmdOn, double kp, double ki, double kd){
    _commandOn = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

PID::PID(bool inCmdOn, double kp, double ki, double kd, uint16_t sampTime){
    _commandOn = inCmdOn;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _sampleTimeMilliS = sampTime;
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
    if(!_autoOn){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pidTimer.finished()){
        _output = _computePID(input);
        // Start the timer again
        _pidTimer.start(_sampleTimeMilliS);
    }
}

// This version of update adds the PID output to the command value, for velocity control
void PID::update(double inCommand, double input){
    // If the PID is turned off do nothing
    if(!_autoOn){return;}

    // Update the PID output on a fixed interval based on our timer
    if(_pidTimer.finished()){
        
        double outPID = _computePID(input);
        outPID = inCommand + outPID;
        _output = constrain(outPID,_outMin,_outMax);

        // Start the timer again
        _pidTimer.start(_sampleTimeMilliS);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void PID::setOutput(double output){
    _output = output;

    // Reset the current output based on the new limits
    _output = constrain(_output,_outMin,_outMax);  
    // Reset the integral term based on the new output limits
    _intTerm = _constrainByCommandMode(_intTerm);   
}

void PID::setPIDGains(double kp, double ki, double kd){
    if(kp<0 || ki<0 || kd<0){return;}

    double sampleTimeInSec = ((double)_sampleTimeMilliS)/1000;
    _kp = kp;
    _ki = ki * sampleTimeInSec;
    _kd = kd / sampleTimeInSec;

    if(_controllerDir == PID_REVERSE){
        _kp = (0.0 - _kp);
        _ki = (0.0 - _ki);
        _kd = (0.0 - _kd);
    }
}

void PID::setPGainOnly(double kp){
    if(kp<0){return;}

    _kp = kp;

    if(_controllerDir == PID_REVERSE){
        _kp = (0.0 - _kp);
    }
}

void PID::setSampleTime(int newSampleTime){
    if (newSampleTime > 0){
        double ratio  = (double)newSampleTime
                        / (double)_sampleTimeMilliS;
        _ki *= ratio;
        _kd /= ratio;
        _sampleTimeMilliS = (unsigned long)newSampleTime;
    }
}

void PID::setOutputLimits(double outMin, double outMax){
    // Error check that the limits are the correct way round
    if(outMin > outMax){return;}

    // Store the new limits
    _outMin = outMin;
    _outMax = outMax;

    // Reset the current output based on the new limits
    _output = constrain(_output,_outMin,_outMax);  
    // Reset the integral term based on the new output limits
    _intTerm = _constrainByCommandMode(_intTerm);   
}

void PID::setCommandLimits(double cmdMin, double cmdMax){
    // Error check that the limits are the correct way round
    if(cmdMin > cmdMax){return;}

    // Store the new limits
    _cmdMin = cmdMin;
    _cmdMax = cmdMax;

    // Reset the current output based on the new limits
    _output = constrain(_output,_outMin,_outMax);  
    // Reset the integral term based on the new output limits
    _intTerm = _constrainByCommandMode(_intTerm);  
}

void PID::setControllerOn(uint8_t inFlag){
    bool onFlag = (inFlag == PID_ON);
    // If we go from manual to auto then initialise PID
    if(onFlag && !_autoOn){
        _intialise();
    }
    _autoOn = onFlag;
}

void PID::setControllerDir(uint8_t inDir){
    // If we are changing direction flip the sign of the gains
    if(inDir != _controllerDir){
        _kp = (0.0 - _kp);
        _ki = (0.0 - _ki);
        _kd = (0.0 - _kd);
    }
    _controllerDir = inDir;
}

//---------------------------------------------------------------------------
// Private Functions

// Used when turning the PID on 
void PID::_intialise(){
    _lastInput = _input;
    _intTerm = _output;
    // Reset the integral term based on the new output limits
    _intTerm = _constrainByCommandMode(_intTerm);
    if (_commandOn){
        _input = 0.0;
        _lastInput = 0.0;
        _intTerm = 0.0;
        _output = 0.0;
    }
    // Reset the integral term based on the new output limits
    _intTerm = _constrainByCommandMode(_intTerm);
}

// main part of the update method used for overloading
double PID::_computePID(double input){
    // Compute the error between the input and the set point
    _error = _setPoint - input;
    _propTerm = _kp*_error;

    // Calculate the integral term using the cummulative error
    _intTerm += (_ki*_error);
    // Clamp the integral term to avoid windup
    _intTerm = _constrainByCommandMode(_intTerm);

    // Calculate the difference in input and use this for the derivative term
    double inputDiff = (input - _lastInput);
    _derivTerm = _kd*inputDiff;

    // Compute PID output using the previous error terms
    double outPID = _propTerm+_intTerm+_derivTerm;
    // Clamp the output to avoid unrealistic values
    outPID = _constrainByCommandMode(outPID);

    // Save variables for next update and return the command
    _lastInput = input;
    return outPID;
}

double PID::_constrainByCommandMode(double inVal){
    if (_commandOn){
        return constrain(inVal,_cmdMin,_cmdMax);
    }
    else{
        return constrain(inVal,_outMin,_outMax);  
    }
}