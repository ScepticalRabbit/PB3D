//==============================================================================
// PB3D: A 3D printed pet robot
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
  PID(bool cmd_on, float kp, float ki, float kd);
  PID(bool cmd_on, float kp, float ki, float kd, uint16_t sample_time);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP

  // This version directly returns the output, useful for position control
  void update(float input);

  // This version of update adds the PID output to the command value, for velocity control
  void update(float command, float input);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void set_output(float output);
  void set_PID_gains(float kp, float ki, float kd);
  void set_Pgain_only(float kp);
  void set_sample_time(int sample_time);
  void set_output_limits(float out_min, float out_max);
  void set_command_limits(float cmd_min, float cmd_max);
  void set_controller_on(uint8_t on_flag);
  void set_controller_dir(uint8_t direction);

  float get_output(){return _output;}
  float get_error(){return _error;}
  void set_set_point(float set_point){_set_point = set_point;}
  float get_set_point(){return _set_point;}
  bool get_controller_on(){return _auto_on;}

  float get_prop_term(){return _prop_term;}
  float get_int_term(){return _int_term;}
  float get_deriv_term(){return _deriv_term;}

private:
  void _intialise();
  float _compute_PID(float input);
  float _constrain_by_command_mode(float val);

  Timer _pid_timer = Timer();
  bool _auto_on = false;
  bool _command_on = true;
  float _kp = 0.1, _ki = 0.0, _kd = 0.0;
  float _last_input = 0.0;
  float _prop_term = 0.0, _int_term = 0.0, _deriv_term = 0.0;
  float _input = 0.0, _set_point = 0.0;
  float _output = 0.0;
  float _out_min = 0.0, _out_max = 255.0;
  float _cmd_min = -127.0, _cmd_max = 127.0;
  float _error = 0.0;
  uint16_t _sample_time_ms = 10;
  uint8_t _controller_dir = PID_DIRECT;
};
#endif // PID_H
