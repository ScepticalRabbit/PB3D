//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_LOOK_H
#define MOVE_LOOK_H

#include <Arduino.h>

class MoveLook{
public:
    MoveLook(){};
    
    void look_around();
    void force_look_move();
    void reset_look();

    // LOOK - Diagnostics
    bool look_is_moving(){return _look_move_switch;}
    bool look_is_paused(){return !_look_move_switch;}

    // LOOK - Getters
    bool get_look_move_switch(){return _look_move_switch;}
    bool get_look_finished(){return (_look_cur_ang >= _look_num_angs);}
    uint8_t get_look_curr_ang_ind(){return _look_cur_ang;}
    uint8_t get_look_num_angs(){return _look_num_angs;}
    uint8_t get_look_max_angs(){return _look_max_angs;}
    float get_look_ang_from_ind(uint8_t index){return _look_angles[index];}
    uint16_t get_look_move_time(){return _look_move_time;}
    uint16_t get_look_pause_time(){return _look_pause_time;}
    uint16_t get_look_tot_time(){return _look_tot_time;}

private:

};
#endif
