//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "CollisionStrategy.h"

EDangerCode CollisionAvoidBasic::get_collision_code(int16_t range){

    if(range < 0 ){return DANGER_NONE;}

    if(range <= _close){return DANGER_CLOSE;}
    else if(range <= _far){return DANGER_FAR;}
    else{return DANGER_NONE;}
}

EDangerCode CollisionAvoidWithSlow::get_collision_code(int16_t range){

    if(range < 0 ){return DANGER_NONE;}

    if(range <= _close){return DANGER_CLOSE;}
    else if(range <= _far){return DANGER_FAR;}
    else if(range <= _slow){return DANGER_SLOW;}
    else{return DANGER_NONE;}
}

EDangerCode CliffAvoid::get_collision_code(int16_t range){

    if(range < 0 ){return DANGER_NONE;}

    if(range >= _close){return DANGER_FAR;}
    else if(range >= _far){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}

EDangerCode CollisionCliffAvoid::get_collision_code(int16_t range){
    if(range < 0 ){return DANGER_NONE;}

    if(range >= _cliff_close){return DANGER_FAR;}
    else if(range <= _col_close){return DANGER_CLOSE;}
    else if(range <= _col_far){return DANGER_FAR;}
    else if(range >= _cliff_far){return DANGER_CLOSE;}
    else{return DANGER_NONE;}
}