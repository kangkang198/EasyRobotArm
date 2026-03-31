#include "robotarm.h"

#ifdef __cplusplus
extern "C" {
#endif

extern RobotArm robotarm;

void shellForwardKinematicsControl(int A,int B,int C)
{
    robotarm.forwardKinematicsControl(A,B,C);
}

void shellForwardKinematicsControl2(int A,int B,int C)
{
    robotarm.forwardKinematicsControl2(A,B,C);
}

void shellBackwardKinematicsControl(int X,int Y,int Z)
{
    robotarm.backwardKinematicsControl(X,Y,Z);
}

void shellBackwardKinematicsControl2(int X,int Y,int Z)
{
    robotarm.backwardKinematicsControl2(X,Y,Z);
}

void shellJawControl(int X,int Y,int Z)
{
    robotarm.jawControl(X,Y,Z);
}

void shellServoTest(int id)
{
    robotarm.servoTest(id);
}

#ifdef __cplusplus
} /* extern "C" */
#endif