#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/StateData.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef STATEDATA_H
#define STATEDATA_H
    #include <Arduino.h>

    #include "PB3DConstants.h"

    // SELECT STATE DATA TYPE
    //#define STATEDATA_DEF
    //#define STATEDATA_NAV
    #define STATEDATA_LASTCOL

    #define STATEDATA_UPD_TIME 101 // milli-seconds

   //---------------------------------------------------------------------------
   #if defined(STATEDATA_LASTCOL)
        struct stateData_t{
            uint32_t onTime;
            uint8_t check_bumpers[BUMP_COUNT];
            uint8_t check_lasers[LASER_COUNT];
            int16_t laser_range_array[LASER_COUNT];
            uint8_t laser_status_array[LASER_COUNT];
            uint8_t escape_count;
            float escape_dist;
            float escape_angle;
        };

        union dataPacket_t{
            stateData_t state;
            byte dataPacket[sizeof(stateData_t)];
        };

        #define PACKET_SIZE sizeof(stateData_t)

        void _init_state_data(dataPacket_t* in_state){
            in_state->state.onTime = 0;

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

        void _print_state_data(dataPacket_t* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: ")); Serial.print(in_state->state.onTime); Serial.print(F("; "));
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

        void _serial_log_data(dataPacket_t* in_state){

            Serial.print(in_state->state.onTime); Serial.print(",");

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
        struct stateData_t{
            // TIME
            uint32_t onTime;
            // MOVE
            float wheelSpeedL;
            float wheelSpeedR;
            // IMU
            float IMUHead;
            float IMUPitch;
            float IMURoll;
            // Navigation
            float navPosX;
            float navPosY;
            float navVelX;
            float navVelY;
            float navVelC;
            float navHead;
        };

        union dataPacket_t{
        stateData_t state;
        byte dataPacket[sizeof(stateData_t)];
        };

        #define PACKET_SIZE sizeof(stateData_t)

        void _init_state_data(dataPacket_t* in_state){
            // TIME
            in_state->state.onTime = 0;
            // MOVE
            in_state->state.wheelSpeedL = 0.0;
            in_state->state.wheelSpeedR = 0.0;
            // IMU
            in_state->state.IMUHead = 0.0;
            in_state->state.IMUPitch = 0.0;
            in_state->state.IMURoll = 0.0;
            // Navigation
            in_state->state.navPosX = 0.0;
            in_state->state.navPosY = 0.0;
            in_state->state.navVelX = 0.0;
            in_state->state.navVelY = 0.0;
            in_state->state.navVelC = 0.0;
            in_state->state.navHead = 0.0;
        }

        void _print_state_data(dataPacket_t* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: ")); Serial.print(in_state->state.onTime); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("W_SpeedL: ")); Serial.print(in_state->state.wheelSpeedL); Serial.print(F("; "));
            Serial.print(F("W_SpeedR: ")); Serial.print(in_state->state.wheelSpeedR); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("IMU,H: ")); Serial.print(in_state->state.IMUHead); Serial.print(F("; "));
            Serial.print(F(" P: ")); Serial.print(in_state->state.IMUPitch); Serial.print(F("; "));
            Serial.print(F(" R: ")); Serial.print(in_state->state.IMURoll); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("NAV,PX: ")); Serial.print(in_state->state.navPosX); Serial.print(F("; "));
            Serial.print(F(" PY: ")); Serial.print(in_state->state.navPosX); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("NAV,VC: ")); Serial.print(in_state->state.navVelC); Serial.print(F("; "));
            Serial.print(F(" VX: ")); Serial.print(in_state->state.navVelX); Serial.print(F("; "));
            Serial.print(F(" VY: ")); Serial.print(in_state->state.navVelX); Serial.print(F("; "));

            Serial.println();

            Serial.println(F("----------------------------------------"));
            Serial.println();
        }
    //---------------------------------------------------------------------------
    #else // Default state data packet
        struct stateData_t{
            // TIME
            uint32_t onTime;
            // MOOD
            int8_t mood;
            int8_t moodScore;
            // TASK
            int8_t task;
            // MOVE
            int8_t moveBasic;
            int8_t moveCompound;
            bool escapeFlag;
            float set_forward_speed;
            float wheelSpeedL;
            float wheelSpeedR;
            int32_t wheelECountL;
            int32_t wheelECountR;
            // COLLISON
            bool colFlag;
            bool colBMPRs;
            bool colUSR;
            bool colLSRL;
            bool colLSRR;
            bool colLSRB;
            bool colLSRU;
            bool colLSRD;
            int16_t colUSRRng;
            int16_t colLSRLRng;
            int16_t colLSRRRng;
            int16_t colLSRBRng;
            int16_t colLSRURng;
            int16_t colLSRDRng;
        };

        union dataPacket_t{
        stateData_t state;
        byte dataPacket[sizeof(stateData_t)];
        };

        #define PACKET_SIZE sizeof(stateData_t)

        void _init_state_data(dataPacket_t* in_state){
            // TIME
            in_state->state.onTime = 0;
            // MOOD
            in_state->state.mood = 0;
            in_state->state.moodScore = 0;
            // TASK
            in_state->state.task = 0;
            // MOVE
            in_state->state.moveBasic = 0;
            in_state->state.moveCompound = 0;
            in_state->state.escapeFlag = false;
            in_state->state.set_forward_speed = 0.0;
            in_state->state.wheelSpeedL = 0.0;
            in_state->state.wheelSpeedR = 0.0;
            in_state->state.wheelECountL = 0;
            in_state->state.wheelECountR = 0;
            // COLLISON - Latches
            in_state->state.colFlag = false;
            in_state->state.colBMPRs = false;
            in_state->state.colUSR = false;
            in_state->state.colLSRL = false;
            in_state->state.colLSRR = false;
            in_state->state.colLSRB = false;
            in_state->state.colLSRU = false;
            in_state->state.colLSRD = false;
            // COLLISION - Ranges
            in_state->state.colUSRRng = 0;
            in_state->state.colLSRLRng = 0;
            in_state->state.colLSRRRng = 0;
            in_state->state.colLSRBRng = 0;
            in_state->state.colLSRURng = 0;
            in_state->state.colLSRDRng = 0;
        }

        void _print_state_data(dataPacket_t* in_state){
            Serial.println();
            Serial.println(F("----------------------------------------"));

            Serial.print(F("Time: ")); Serial.print(in_state->state.onTime); Serial.print(F("; "));
            Serial.print(F("Mood: ")); Serial.print(in_state->state.mood); Serial.print(F("; "));
            Serial.print(F("MoodSc: ")); Serial.print(in_state->state.moodScore); Serial.print(F("; "));
            Serial.print(F("TaskManager: ")); Serial.print(in_state->state.task); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("MoveB: ")); Serial.print(in_state->state.moveBasic); Serial.print(F("; "));
            Serial.print(F("MoveC: ")); Serial.print(in_state->state.moveCompound); Serial.print(F("; "));
            Serial.print(F("MoveEsc: ")); Serial.print(in_state->state.escapeFlag); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("FwdSpeed: ")); Serial.print(in_state->state.set_forward_speed); Serial.print(F("; "));
            Serial.print(F("W_SpeedL: ")); Serial.print(in_state->state.wheelSpeedL); Serial.print(F("; "));
            Serial.print(F("W_SpeedR: ")); Serial.print(in_state->state.wheelSpeedR); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("W_EncCL: ")); Serial.print(in_state->state.wheelECountL); Serial.print(F("; "));
            Serial.print(F("W_EncCR: ")); Serial.print(in_state->state.wheelECountR); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("ColALL: ")); Serial.print(in_state->state.colFlag); Serial.print(F("; "));
            Serial.print(F("ColBmp: ")); Serial.print(in_state->state.colBMPRs); Serial.print(F("; "));
            Serial.print(F("ColUsr: ")); Serial.print(in_state->state.colUSR); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("ColLsr: L: ")); Serial.print(in_state->state.colLSRL); Serial.print(F("; "));
            Serial.print(F("R: ")); Serial.print(in_state->state.colLSRR); Serial.print(F("; "));
            Serial.print(F("B: ")); Serial.print(in_state->state.colLSRB); Serial.print(F("; "));
            Serial.print(F("U: ")); Serial.print(in_state->state.colLSRU); Serial.print(F("; "));
            Serial.print(F("D: ")); Serial.print(in_state->state.colLSRD); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("Rngs(mm): US: ")); Serial.print(in_state->state.colUSRRng); Serial.print(F("; "));
            Serial.print(F("L: ")); Serial.print(in_state->state.colLSRLRng); Serial.print(F("; "));
            Serial.print(F("R: ")); Serial.print(in_state->state.colLSRRRng); Serial.print(F("; "));
            Serial.println();

            Serial.print(F("Rngs(mm): B: ")); Serial.print(in_state->state.colLSRBRng); Serial.print(F("; "));
            Serial.print(F("U: ")); Serial.print(in_state->state.colLSRURng); Serial.print(F("; "));
            Serial.print(F("D: ")); Serial.print(in_state->state.colLSRDRng); Serial.print(F("; "));
            Serial.println();

            Serial.println(F("----------------------------------------"));
            Serial.println();
        }
    #endif
#endif // STATEDATA_H

// DEFAULT FUNCTIONS AND STATE
/*
    typedef struct stateData_t{

    };

    typedef union dataPacket_t{
      stateData_t state;
      byte dataPacket[sizeof(stateData_t)];
    };

    #define PACKET_SIZE sizeof(stateData_t)

    void _init_state_data(dataPacket_t* in_state){

    }

    void _print_state_data(dataPacket_t* in_state){

    }
*/