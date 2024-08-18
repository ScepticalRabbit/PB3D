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

// SELECT STATE DATA TYPE
//#define STATEDATA_DEF
//#define STATEDATA_NAV
#define STATEDATA_LASTCOL

#define STATEDATA_UPD_TIME 101 // milli-seconds

#if defined(STATEDATA_LASTCOL)
    struct stateData_t{
        // TIME
        uint32_t onTime;
        // LAST COLLISION
        uint8_t check_vec[7];
        uint16_t ultrasonic_range;
        uint16_t LSRRangeL, LSRRangeR;
        uint16_t LSRRangeU, LSRRangeD;
        float escape_count, escape_dist, escape_angle;
    };

    union dataPacket_t{
      stateData_t state;
      byte dataPacket[sizeof(stateData_t)];
    };

    #define PACKET_SIZE sizeof(stateData_t)

    void _init_state_data(dataPacket_t* inState){
        // TIME
        inState->state.onTime = 0;
        // LAST COLLISION
        for(uint8_t ii=0;ii<7;ii++){
            inState->state.check_vec[ii] = 0;
        }
        inState->state.ultrasonic_range = 0;
        inState->state.LSRRangeL = 0;
        inState->state.LSRRangeR = 0;
        inState->state.LSRRangeU = 0;
        inState->state.LSRRangeD = 0;
        inState->state.escape_count = 0;
        inState->state.escape_dist = 0.0;
        inState->state.escape_angle = 0.0;
    }

    void _print_state_data(dataPacket_t* inState){
        Serial.println();
        Serial.println(F("----------------------------------------"));

        Serial.print(F("Time: ")); Serial.print(inState->state.onTime); Serial.print(F("; "));
        Serial.println();

        Serial.println(F("CheckVec=[BL,BR,US,LL,LR,LU,LD,]"));
        Serial.print("CheckVec=[");
        for(uint8_t ii=0;ii<7;ii++){
            Serial.print(" ");Serial.print(inState->state.check_vec[ii]);Serial.print(",");
        }
        Serial.println("]");

        Serial.print("US="); Serial.print(inState->state.ultrasonic_range); Serial.print("mm, ");
        Serial.print("LL="); Serial.print(inState->state.LSRRangeL); Serial.print("mm, ");
        Serial.print("LR="); Serial.print(inState->state.LSRRangeR); Serial.print("mm");
        Serial.println();
        Serial.print("LU="); Serial.print(inState->state.LSRRangeU); Serial.print("mm, ");
        Serial.print("LD="); Serial.print(inState->state.LSRRangeD); Serial.print("mm");
        Serial.println();
        Serial.print("Esc,Count="); Serial.print(inState->state.escape_count);
        Serial.print(", Dist="); Serial.print(inState->state.escape_dist);
        Serial.print(", Angle="); Serial.print(inState->state.escape_angle);
        Serial.println();

        Serial.println(F("----------------------------------------"));
        Serial.println();
    }

    void _serial_log_data(dataPacket_t* inState){
        Serial.print(inState->state.onTime); Serial.print(",");
        for(uint8_t ii=0;ii<7;ii++){
            Serial.print(inState->state.check_vec[ii]);Serial.print(",");
        }
        Serial.print(inState->state.ultrasonic_range); Serial.print(",");
        Serial.print(inState->state.LSRRangeL); Serial.print(",");
        Serial.print(inState->state.LSRRangeR); Serial.print(",");
        Serial.print(inState->state.LSRRangeU); Serial.print(",");
        Serial.print(inState->state.LSRRangeD); Serial.print(",");

        Serial.print(inState->state.escape_count); Serial.print(",");
        Serial.print(inState->state.escape_dist); Serial.print(",");
        Serial.print(inState->state.escape_angle); Serial.print(",");
        Serial.println();
    }

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

    void _initStateData(dataPacket_t* inState){
        // TIME
        inState->state.onTime = 0;
        // MOVE
        inState->state.wheelSpeedL = 0.0;
        inState->state.wheelSpeedR = 0.0;
        // IMU
        inState->state.IMUHead = 0.0;
        inState->state.IMUPitch = 0.0;
        inState->state.IMURoll = 0.0;
        // Navigation
        inState->state.navPosX = 0.0;
        inState->state.navPosY = 0.0;
        inState->state.navVelX = 0.0;
        inState->state.navVelY = 0.0;
        inState->state.navVelC = 0.0;
        inState->state.navHead = 0.0;
    }

    void _print_state_data(dataPacket_t* inState){
        Serial.println();
        Serial.println(F("----------------------------------------"));

        Serial.print(F("Time: ")); Serial.print(inState->state.onTime); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("W_SpeedL: ")); Serial.print(inState->state.wheelSpeedL); Serial.print(F("; "));
        Serial.print(F("W_SpeedR: ")); Serial.print(inState->state.wheelSpeedR); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("IMU,H: ")); Serial.print(inState->state.IMUHead); Serial.print(F("; "));
        Serial.print(F(" P: ")); Serial.print(inState->state.IMUPitch); Serial.print(F("; "));
        Serial.print(F(" R: ")); Serial.print(inState->state.IMURoll); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("NAV,PX: ")); Serial.print(inState->state.navPosX); Serial.print(F("; "));
        Serial.print(F(" PY: ")); Serial.print(inState->state.navPosX); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("NAV,VC: ")); Serial.print(inState->state.navVelC); Serial.print(F("; "));
        Serial.print(F(" VX: ")); Serial.print(inState->state.navVelX); Serial.print(F("; "));
        Serial.print(F(" VY: ")); Serial.print(inState->state.navVelX); Serial.print(F("; "));

        Serial.println();

        Serial.println(F("----------------------------------------"));
        Serial.println();
    }

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

    void _initStateData(dataPacket_t* inState){
        // TIME
        inState->state.onTime = 0;
        // MOOD
        inState->state.mood = 0;
        inState->state.moodScore = 0;
        // TASK
        inState->state.task = 0;
        // MOVE
        inState->state.moveBasic = 0;
        inState->state.moveCompound = 0;
        inState->state.escapeFlag = false;
        inState->state.set_forward_speed = 0.0;
        inState->state.wheelSpeedL = 0.0;
        inState->state.wheelSpeedR = 0.0;
        inState->state.wheelECountL = 0;
        inState->state.wheelECountR = 0;
        // COLLISON - Latches
        inState->state.colFlag = false;
        inState->state.colBMPRs = false;
        inState->state.colUSR = false;
        inState->state.colLSRL = false;
        inState->state.colLSRR = false;
        inState->state.colLSRB = false;
        inState->state.colLSRU = false;
        inState->state.colLSRD = false;
        // COLLISION - Ranges
        inState->state.colUSRRng = 0;
        inState->state.colLSRLRng = 0;
        inState->state.colLSRRRng = 0;
        inState->state.colLSRBRng = 0;
        inState->state.colLSRURng = 0;
        inState->state.colLSRDRng = 0;
    }

    void _print_state_data(dataPacket_t* inState){
        Serial.println();
        Serial.println(F("----------------------------------------"));

        Serial.print(F("Time: ")); Serial.print(inState->state.onTime); Serial.print(F("; "));
        Serial.print(F("Mood: ")); Serial.print(inState->state.mood); Serial.print(F("; "));
        Serial.print(F("MoodSc: ")); Serial.print(inState->state.moodScore); Serial.print(F("; "));
        Serial.print(F("TaskManager: ")); Serial.print(inState->state.task); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("MoveB: ")); Serial.print(inState->state.moveBasic); Serial.print(F("; "));
        Serial.print(F("MoveC: ")); Serial.print(inState->state.moveCompound); Serial.print(F("; "));
        Serial.print(F("MoveEsc: ")); Serial.print(inState->state.escapeFlag); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("FwdSpeed: ")); Serial.print(inState->state.set_forward_speed); Serial.print(F("; "));
        Serial.print(F("W_SpeedL: ")); Serial.print(inState->state.wheelSpeedL); Serial.print(F("; "));
        Serial.print(F("W_SpeedR: ")); Serial.print(inState->state.wheelSpeedR); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("W_EncCL: ")); Serial.print(inState->state.wheelECountL); Serial.print(F("; "));
        Serial.print(F("W_EncCR: ")); Serial.print(inState->state.wheelECountR); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("ColALL: ")); Serial.print(inState->state.colFlag); Serial.print(F("; "));
        Serial.print(F("ColBmp: ")); Serial.print(inState->state.colBMPRs); Serial.print(F("; "));
        Serial.print(F("ColUsr: ")); Serial.print(inState->state.colUSR); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("ColLsr: L: ")); Serial.print(inState->state.colLSRL); Serial.print(F("; "));
        Serial.print(F("R: ")); Serial.print(inState->state.colLSRR); Serial.print(F("; "));
        Serial.print(F("B: ")); Serial.print(inState->state.colLSRB); Serial.print(F("; "));
        Serial.print(F("U: ")); Serial.print(inState->state.colLSRU); Serial.print(F("; "));
        Serial.print(F("D: ")); Serial.print(inState->state.colLSRD); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("Rngs(mm): US: ")); Serial.print(inState->state.colUSRRng); Serial.print(F("; "));
        Serial.print(F("L: ")); Serial.print(inState->state.colLSRLRng); Serial.print(F("; "));
        Serial.print(F("R: ")); Serial.print(inState->state.colLSRRRng); Serial.print(F("; "));
        Serial.println();

        Serial.print(F("Rngs(mm): B: ")); Serial.print(inState->state.colLSRBRng); Serial.print(F("; "));
        Serial.print(F("U: ")); Serial.print(inState->state.colLSRURng); Serial.print(F("; "));
        Serial.print(F("D: ")); Serial.print(inState->state.colLSRDRng); Serial.print(F("; "));
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

    void _init_state_data(dataPacket_t* inState){

    }

    void _print_state_data(dataPacket_t* inState){

    }
*/