//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_STATEDATA_H
#define PB3D_STATEDATA_H
    #include <Arduino.h>

    #include <PB3DConstants.h>

    // SELECT STATE DATA TYPE
    #define STATEDATA_DEF
    //#define STATEDATA_NAV
    //#define STATEDATA_LASTCOL

    #define STATEDATA_UPD_TIME 101 // milli-seconds

   //---------------------------------------------------------------------------
   #if defined(STATEDATA_LASTCOL)
        struct SStateData{
            uint32_t on_time;
            uint8_t check_bumpers[BUMP_COUNT];
            uint8_t check_lasers[LASER_COUNT];
            int16_t laser_range_array[LASER_COUNT];
            uint8_t laser_status_array[LASER_COUNT];
            uint8_t escape_count;
            float escape_dist;
            float escape_angle;
        };

        union UDataPacket{
            SStateData state;
            byte data_packet[sizeof(SStateData)];
        };

        #define PACKET_SIZE sizeof(SStateData)

        void _init_state_data(UDataPacket* in_state){
            in_state->state.on_time = 0;

            for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
                in_state->state.check_bumpers[ii] = DANGER_NONE;
            }
            for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
                in_state->state.check_lasers[ii] = DANGER_NONE;
                in_state->state.laser_range_array[ii] = 0;
                in_state->state.laser_status_array[ii] = 0;
            }

            in_state->state.escape_count = 0;
            in_state->state.escape_dist = 0.0;
            in_state->state.escape_angle = 0.0;
        }

        void _print_state_data(UDataPacket* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: ")); Serial.print(in_state->state.on_time); Serial.print(F("; "));
            Serial.println();

            Serial.print("bumper_checks=[");
            for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
                Serial.print(" ");
                Serial.print(in_state->state.check_bumpers[ii]);
                Serial.print(",");
            }
            Serial.println("]");

            Serial.print("laser_checks=[");
            for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
                Serial.print(" ");
                Serial.print(in_state->state.check_lasers[ii]);
                Serial.print(",");
            }
            Serial.println("]");

            Serial.print("Esc,Count="); Serial.print(in_state->state.escape_count);
            Serial.print(", Dist="); Serial.print(in_state->state.escape_dist);
            Serial.print(", Angle="); Serial.print(in_state->state.escape_angle);
            Serial.println();

            Serial.println(F("----------------------------------------"));
            Serial.println();
        }

        void _serial_log_data(UDataPacket* in_state){

            Serial.print(in_state->state.on_time); Serial.print(",");

            for(uint8_t ii=0;ii<BUMP_COUNT;ii++){
                Serial.print(in_state->state.check_bumpers[ii]);
                Serial.print(",");
            }

            for(uint8_t ii=0;ii<LASER_COUNT;ii++){
                Serial.print(in_state->state.check_lasers[ii]);
                Serial.print(",");
            }
            for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
                Serial.print(in_state->state.laser_range_array[ii]);
                Serial.print(",");
            }

            Serial.print(in_state->state.escape_count); Serial.print(",");
            Serial.print(in_state->state.escape_dist); Serial.print(",");
            Serial.print(in_state->state.escape_angle); Serial.print(",");
            Serial.println();
        }
    //---------------------------------------------------------------------------
    #elif defined(STATEDATA_NAV)
        struct SStateData{
            // TIME
            uint32_t on_time;
            // MOVE
            float wheel_speed_left;
            float wheel_speed_right;
            // IMU
            float IMU_heading;
            float IMU_pitch;
            float IMU_roll;
            // Navigation
            float nav_pos_x;
            float nav_pos_y;
            float nav_vel_x;
            float nav_vel_y;
            float nav_vel_c;
            float nav_head;
        };

        union UDataPacket{
            SStateData state;
            byte data_packet[sizeof(SStateData)];
        };

        #define PACKET_SIZE sizeof(SStateData)

        void _init_state_data(UDataPacket* in_state){
            // TIME
            in_state->state.on_time = 0;
            // MOVE
            in_state->state.wheel_speed_left = 0.0;
            in_state->state.wheel_speed_right = 0.0;
            // IMU
            in_state->state.IMU_heading = 0.0;
            in_state->state.IMU_pitch = 0.0;
            in_state->state.IMU_roll = 0.0;
            // Navigation
            in_state->state.nav_pos_x = 0.0;
            in_state->state.nav_pos_y = 0.0;
            in_state->state.nav_vel_x = 0.0;
            in_state->state.nav_vel_y = 0.0;
            in_state->state.nav_vel_c = 0.0;
            in_state->state.nav_head = 0.0;
        }

        void _print_state_data(UDataPacket* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: "));
            Serial.print(in_state->state.on_time);
            Serial.print(F("; "));
            Serial.println();

            Serial.print(F("W_SpeedL: "));
            Serial.print(in_state->state.wheel_speed_left);
            Serial.print(F("; "));
            Serial.print(F("W_SpeedR: "));
            Serial.print(in_state->state.wheel_speed_right);
            Serial.print(F("; "));
            Serial.println();

            Serial.print(F("IMU,H: "));
            Serial.print(in_state->state.IMU_heading);
            Serial.print(F("; "));
            Serial.print(F(" P: "));
            Serial.print(in_state->state.IMU_pitch);
            Serial.print(F("; "));
            Serial.print(F(" R: "));
            Serial.print(in_state->state.IMU_roll);
            Serial.print(F("; "));
            Serial.println();

            Serial.print(F("NAV,PX: "));
            Serial.print(in_state->state.nav_pos_x);
            Serial.print(F("; "));
            Serial.print(F(" PY: "));
            Serial.print(in_state->state.nav_pos_x);
            Serial.print(F("; "));
            Serial.println();

            Serial.print(F("NAV,VC: "));
            Serial.print(in_state->state.nav_vel_c);
            Serial.print(F("; "));
            Serial.print(F(" VX: "));
            Serial.print(in_state->state.nav_vel_x);
            Serial.print(F("; "));
            Serial.print(F(" VY: "));
            Serial.print(in_state->state.nav_vel_x);
            Serial.print(F("; "));

            Serial.println();

            Serial.println(F("----------------------------------------"));
            Serial.println();
        }
    //---------------------------------------------------------------------------
    #else // Default state data packet
        struct SStateData{
            // TIME
            uint32_t on_time;
            // MOOD
            int8_t mood;
            int8_t mood_score;
            // TASK
            int8_t task;
            // MOVE
            int8_t move_basic;
            int8_t move_compound;
            float set_forward_speed;
            float wheel_speed_left;
            float wheel_speed_right;
            int32_t wheel_encoder_count_left;
            int32_t wheel_encoder_count_right;
            // COLLISON
            uint8_t check_bumpers[BUMP_COUNT];
            uint8_t check_lasers[LASER_COUNT];
            int16_t laser_range_array[LASER_COUNT];
        };

        union UDataPacket{
            SStateData state;
            byte data_packet[sizeof(SStateData)];
        };

        #define PACKET_SIZE sizeof(SStateData)

        void _init_state_data(UDataPacket* in_state){
            // TIME
            in_state->state.on_time = 0;
            // MOOD
            in_state->state.mood = 0;
            in_state->state.mood_score = 0;
            // TASK
            in_state->state.task = 0;
            // MOVE
            in_state->state.move_basic = 0;
            in_state->state.move_compound = 0;
            in_state->state.set_forward_speed = 0.0;
            in_state->state.wheel_speed_left = 0.0;
            in_state->state.wheel_speed_right = 0.0;
            in_state->state.wheel_encoder_count_left = 0;
            in_state->state.wheel_encoder_count_right = 0;
            // COLLISON
            for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
                in_state->state.check_bumpers[ii] = DANGER_NONE;
            }
            for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
                in_state->state.check_lasers[ii] = DANGER_NONE;
            }
        }

        void _print_state_data(UDataPacket* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: ")); Serial.print(in_state->state.on_time); Serial.print(F("; "));
            Serial.print(F("Mood: ")); Serial.print(in_state->state.mood); Serial.print(F("; "));
            Serial.print(F("MoodSc: ")); Serial.print(in_state->state.mood_score); Serial.print(F("; "));
            Serial.print(F("TaskManager: ")); Serial.print(in_state->state.task); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("MoveB: ")); Serial.print(in_state->state.move_basic); Serial.print(F("; "));
            Serial.print(F("MoveC: ")); Serial.print(in_state->state.move_compound); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("FwdSpeed: ")); Serial.print(in_state->state.set_forward_speed); Serial.print(F("; "));
            Serial.print(F("W_SpeedL: ")); Serial.print(in_state->state.wheel_speed_left); Serial.print(F("; "));
            Serial.print(F("W_SpeedR: ")); Serial.print(in_state->state.wheel_speed_right); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("W_EncCL: ")); Serial.print(in_state->state.wheel_encoder_count_left); Serial.print(F("; "));
            Serial.print(F("W_EncCR: ")); Serial.print(in_state->state.wheel_encoder_count_right); Serial.print(F("; "));
            Serial.println();

            //TODO: print laser collision codes and ranges

            Serial.println(F("----------------------------------------"));
            Serial.println();
        }
    #endif
#endif // STATEDATA_H

// DEFAULT FUNCTIONS AND STATE
/*
    typedef struct SStateData{

    };

    typedef union UDataPacket{
      SStateData state;
      byte data_packet[sizeof(SStateData)];
    };

    #define PACKET_SIZE sizeof(SStateData)

    void _init_state_data(UDataPacket* in_state){

    }

    void _print_state_data(UDataPacket* in_state){

    }
*/