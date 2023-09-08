#ifndef COLLISIONESCAPER_H
#define COLLISIONESCAPER_H

#include "Move.h"

#define ESCAPE_NOREV 1
#define ESCAPE_REV 0


class CollisionEscaper{
    public:
        CollisionEscaper(){};

        void setMoveObj(Move* inMove);
        void updateEscapeDecision(uint8_t checkVec[]);
        void setEscapeStart(uint8_t checkVec[]);
        void escape();
        bool getEscapeFlag();
        int8_t getEscapeTurn(uint8_t checkVec[]);

        uint8_t getEscapeCount(){return _escapeCount;}
        float getEscapeDist(){return _escapeDist;}
        float getEscapeAngle(){return _escapeAngle;}

    private:
        float _getRandTurnDir();

        Move* _moveObj = NULL;
        uint8_t _escapeCount = 3;
        float _escapeAngle = 45.0;
        float _escapeDist = 180.0;
        const static uint8_t _escapeNumSteps = 3;
        float _defModTurn = 45.0, _defHardTurn = 90.0;
        float _defRevDist = -180.0;
};

#endif