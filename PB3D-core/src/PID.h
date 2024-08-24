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

#include <PB3DConstants.h>
#include "PB3DTimer.h"


class PID{
public:
  PID(bool cmd_on);
  PID(bool cmd_on, double kp, double ki, double kd);
  PID(bool cmd_on, double kp, double ki, double kd, uint16_t sample_time);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP

  // This version directly returns the output, useful for position control
  void update(double input);

  // This version of update adds the PID output to the command value, for velocity control
  void update(double command, double input);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void set_output(double output);
  void set_PID_gains(double kp, double ki, double kd);
  void set_Pgain_only(double kp);
  void set_sample_time(int sample_time);
  void set_output_limits(double out_min, double out_max);
  void set_command_limits(double cmd_min, double cmd_max);
  void set_controller_on(uint8_t on_flag);
  void set_controller_dir(uint8_t direction);

  double get_output(){return _output;}
  double get_error(){return _error;}
  void set_set_point(double set_point){_set_point = set_point;}
  double get_set_point(){return _set_point;}
  bool get_controller_on(){return _auto_on;}

  double get_prop_term(){return _prop_term;}
  double get_int_term(){return _int_term;}
  double get_deriv_term(){return _deriv_term;}

private:
  // Used when turning the PID on
  void _intialise();
  // main part of the update method used for overloading
  double _compute_PID(double input);
  double _constrain_by_command_mode(double val);

  // PRIVATE VARIABLES
  Timer _pid_timer = Timer();
  bool _auto_on = false;
  bool _command_on = true;
  double _kp = 0.1, _ki = 0.0, _kd = 0.0;
  double _last_input = 0.0;
  double _prop_term = 0.0, _int_term = 0.0, _deriv_term = 0.0;
  double _input = 0.0, _set_point = 0.0;
  double _output = 0.0;
  double _out_min = 0.0, _out_max = 255.0;
  double _cmd_min = -127.0, _cmd_max = 127.0;
  double _error = 0.0;
  uint16_t _sample_time_ms = 10;
  uint8_t _controller_dir = PID_DIRECT;
};
#endif // PID_H
