#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/PID.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Based on code found here:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//------------------------------------------------------------------------------

#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "Timer.h"

#define PID_OFF 0
#define PID_ON 1

#define PID_DIRECT 0
#define PID_REVERSE 1

class PID{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTORS
  PID(bool inCmdOn);
  PID(bool inCmdOn, double kp, double ki, double kd);
  PID(bool inCmdOn, double kp, double ki, double kd, uint16_t sampTime);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP

  // This version directly returns the output, useful for position control
  void update(double input);

  // This version of update adds the PID output to the command value, for velocity control
  void update(double inCommand, double input);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void setOutput(double output);
  void setPIDGains(double kp, double ki, double kd);
  void setPGainOnly(double kp);
  void setSampleTime(int newSampleTime);
  void setOutputLimits(double outMin, double outMax);
  void setCommandLimits(double cmdMin, double cmdMax);
  void setControllerOn(uint8_t inFlag);
  void setControllerDir(uint8_t inDir);

  double get_output(){return _output;}
  double getError(){return _error;}
  void setSetPoint(double setPoint){_setPoint = setPoint;}
  double get_set_point(){return _setPoint;}
  bool getControllerOn(){return _autoOn;}
  double getPropTerm(){return _propTerm;}
  double getIntTerm(){return _intTerm;}
  double getDerivTerm(){return _derivTerm;}

private:
  // Used when turning the PID on
  void _intialise();
  // main part of the update method used for overloading
  double _computePID(double input);
  double _constrainByCommandMode(double inVal);

  // PRIVATE VARIABLES
  Timer _pidTimer = Timer();
  bool _autoOn = false;
  bool _commandOn = true;
  double _kp = 0.1, _ki = 0.0, _kd = 0.0;
  double _lastInput = 0.0;
  double _propTerm = 0.0, _intTerm = 0.0, _derivTerm = 0.0;
  double _input = 0.0, _setPoint = 0.0;
  double _output = 0.0;
  double _outMin = 0.0, _outMax = 255.0;
  double _cmdMin = -127.0, _cmdMax = 127.0;
  double _error = 0.0;
  uint16_t _sampleTimeMilliS = 10;
  uint8_t _controllerDir = PID_DIRECT;
};
#endif // PID_H
