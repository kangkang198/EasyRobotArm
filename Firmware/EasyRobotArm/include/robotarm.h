#ifndef __ROBOTARM_H__
#define __ROBOTARM_H__

#include <ESP32Servo.h>

#define SERVOA_PIN 13
#define SERVOB_PIN 14
#define SERVOC_PIN 21
#define SERVOD_PIN 47

#define SERVO_FREQ 50 //50Hz
#define SERVO_MIN_PERIOD 500 //500us
#define SERVO_MAX_PERIOD 2500 //2500us

#define SERVEA_INIT_ANGLE 90
#define SERVEA_MIN_ANGLE 0
#define SERVEA_MAX_ANGLE 180

#define SERVEB_INIT_ANGLE 90
#define SERVEB_MIN_ANGLE 90
#define SERVEB_MAX_ANGLE 160

#define SERVEC_INIT_ANGLE 120
#define SERVEC_MIN_ANGLE 120
#define SERVEC_MAX_ANGLE 160

#define SERVED_INIT_ANGLE 60
#define SERVED_MIN_ANGLE 60
#define SERVED_MAX_ANGLE 100

//关节角度转舵机角度
#define JA2SA(JA) (JA + 90)
#define JB2SB(JB) (JB + 90)
#define JC2SC(JC,JB) (JC - JB + 90) 

//舵机角度转关节角度
#define SA2JA(SA) (SA - 90)
#define SB2JB(SB) (SB - 90)
#define SC2JC(SC,SB) (SC + SB - 180)

//夹爪开合状态和开合角度
#define JAW_OPEN (0)
#define JAW_CLOSE (1)
#define JAW_OPEN_ANGLE (SERVED_MIN_ANGLE)
#define JAW_CLOSE_ANGLE (SERVED_MAX_ANGLE)

class RobotArm
{
    private:
        Servo ServoA;
        Servo ServoB;
        Servo ServoC;
        Servo ServoD;

        typedef struct
        {
            float A;
            float B;
            float C;
        }Jot_Ang_t;

        typedef struct
        {
            int A;
            int B;
            int C;
            int D; //夹爪的舵机
        }Servo_Ang_t;

        typedef struct
        {
            float X;
            float Y;
            float Z;
        }Jaw_Pos_t;

        enum
        {
            BACKWARD = 0,
            FORWARD = 1
        };

        // 机械臂各轴角度
        Jot_Ang_t LastJot;
        Jot_Ang_t CurJot;
        Jot_Ang_t TarJot;

        // 舵机各轴角度
        Servo_Ang_t LastServo;
        Servo_Ang_t CurServo;
        Servo_Ang_t TarServo;

        // 夹爪的绝对坐标
        Jaw_Pos_t LastJaw;
        Jaw_Pos_t CurJaw;
        Jaw_Pos_t TarJaw;

    public:
        void init();
        void forwardKinematics(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo);
        void backwardKinematics(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo);
        void servoAngleMove(Servo *Servo,int CurAng,int TarAng);
        void updateJawParams(Jaw_Pos_t *LastJaw,Jaw_Pos_t *CurJaw,Jaw_Pos_t *TarJaw);
        void updateJotParams(Jot_Ang_t *LastJot,Jot_Ang_t *CurJot,Jot_Ang_t *TarJot);
        void updateServoParams(Servo_Ang_t *LastServo,Servo_Ang_t *CurServo,Servo_Ang_t *TarServo);
        void updateParams();
        void servoAngleControl(uint8_t dir);
        void jawAngleControl(uint8_t status);
        void forwardKinematicsControl(int A,int B,int C);
        void forwardKinematicsControl2(int A,int B,int C);
        void backwardKinematicsControl(int X,int Y,int Z);
        void backwardKinematicsControl2(int X,int Y,int Z);
        void jawControl(int X,int Y,int Z);
        void showParams(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo);
        void servoLimit(Servo_Ang_t *Servo);
        void jotLimit(Jot_Ang_t *Jot);
        void servoTest(int id);
};

#endif