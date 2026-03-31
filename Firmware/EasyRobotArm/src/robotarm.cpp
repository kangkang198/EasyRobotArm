#include "robotarm.h"

void RobotArm::init()
{
    ServoA.setPeriodHertz(SERVO_FREQ);//50Hz,20ms
    ServoB.setPeriodHertz(SERVO_FREQ);//50Hz,20ms
    ServoC.setPeriodHertz(SERVO_FREQ);//50Hz,20ms
    ServoD.setPeriodHertz(SERVO_FREQ);//50Hz,20ms
    ServoA.attach(SERVOA_PIN,SERVO_MIN_PERIOD,SERVO_MAX_PERIOD);
    ServoB.attach(SERVOB_PIN,SERVO_MIN_PERIOD,SERVO_MAX_PERIOD);
    ServoC.attach(SERVOC_PIN,SERVO_MIN_PERIOD,SERVO_MAX_PERIOD);
    ServoD.attach(SERVOD_PIN,SERVO_MIN_PERIOD,SERVO_MAX_PERIOD);
    ServoA.write(SERVEA_INIT_ANGLE);
    ServoB.write(SERVEB_INIT_ANGLE);
    ServoC.write(SERVEC_INIT_ANGLE);
    ServoD.write(SERVED_INIT_ANGLE);

    TarJot.A = SA2JA(SERVEA_INIT_ANGLE);
    TarJot.B = SA2JA(SERVEB_INIT_ANGLE);
    TarJot.C = SA2JA(SERVEC_INIT_ANGLE);

    CurServo.D = TarServo.D = SERVED_INIT_ANGLE;

    forwardKinematics(&TarJaw,&TarJot,&TarServo);
    
    updateParams();

    log_printf("Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);

    log_printf("Target:\n");
    showParams(&TarJaw,&TarJot,&TarServo);
}

void RobotArm::updateJawParams(Jaw_Pos_t *LastJaw,Jaw_Pos_t *CurJaw,Jaw_Pos_t *TarJaw)
{
  LastJaw->X = CurJaw->X;
  LastJaw->Y = CurJaw->Y;
  LastJaw->Z = CurJaw->Z;

  CurJaw->X = TarJaw->X;
  CurJaw->Y = TarJaw->Y;
  CurJaw->Z = TarJaw->Z;
}

void RobotArm::updateJotParams(Jot_Ang_t *LastJot,Jot_Ang_t *CurJot,Jot_Ang_t *TarJot)
{
  LastJot->A = CurJot->A;
  LastJot->B = CurJot->B;
  LastJot->C = CurJot->C;

  CurJot->A = TarJot->A;
  CurJot->B = TarJot->B;
  CurJot->C = TarJot->C;
}

void RobotArm::updateServoParams(Servo_Ang_t *LastServo,Servo_Ang_t *CurServo,Servo_Ang_t *TarServo)
{
  LastServo->A = CurServo->A;
  LastServo->B = CurServo->B;
  LastServo->C = CurServo->C;
  LastServo->D = CurServo->D;

  CurServo->A = TarServo->A;
  CurServo->B = TarServo->B;
  CurServo->C = TarServo->C;
  CurServo->D = TarServo->D;
}

void RobotArm::showParams(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo)
{
  log_printf("Jaw[X,Y,Z] = [%d,%d,%d]\n",int(Jaw->X),int(Jaw->Y),int(Jaw->Z));
  log_printf("Jot[A,B,C] = [%d,%d,%d]\n",int(Jot->A),int(Jot->B),int(Jot->C));
  log_printf("Servo[A,B,C,D] = [%d,%d,%d,%d]\n",Servo->A,Servo->B,Servo->C,Servo->D);
}

void RobotArm::updateParams()
{
  updateJawParams(&LastJaw,&CurJaw,&TarJaw);
  updateJotParams(&LastJot,&CurJot,&TarJot);
  updateServoParams(&LastServo,&CurServo,&TarServo);
}

//正运算:
//从机械臂关节角度计算出夹爪的坐标
void RobotArm::forwardKinematics(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo)
{
  float LAB = 60.0f;
  float LBC = 72.5f;
  float LCD = 72.5f;
  float L1 = LBC * cos(radians(180-80-Jot->B));
  float L2 = LCD * sin(radians(Jot->C - Jot->B + (180-80-90)));
  float L3 = LBC * sin(radians(180-80-Jot->B));
  float L4 = LCD * cos(radians(Jot->C - Jot->B + (180-80-90)));
  float L5 = 20.0f;
  float L6 = 60.0f;
  float tempX = L1 + L2 + L6;
  float tempY = LAB + L3 - L4 - L5;

  Jaw->X = tempX * sin(radians(Jot->A));
  Jaw->Y = tempY;
  Jaw->Z = tempX * cos(radians(Jot->A));

  Servo->A = (int)(JA2SA(Jot->A));
  Servo->B = (int)(JB2SB(Jot->B));
  Servo->C = (int)(JC2SC(Jot->C,Jot->B));

  servoLimit(Servo);
}

//逆运算:
//从夹爪的坐标计算出机械臂关节角度
void RobotArm::backwardKinematics(Jaw_Pos_t *Jaw,Jot_Ang_t *Jot,Servo_Ang_t *Servo)
{
  float tempX = sqrt(Jaw->X * Jaw->X + Jaw->Z * Jaw->Z);
  float tempY = Jaw->Y;
  float LAB = 60.0f;
  float LBC = 72.5f;
  float LCD = 72.5f;
  float L4 = 20.0f;
  float L5 = 60.0f;
  float L1 = tempX - L5;
  float L2 = tempY - LAB;
  float L3 = sqrt((L2+L4) * (L2+L4) + L1 * L1);
  float tempA,tempB;

  Jot->A = degrees(atan(Jaw->X/Jaw->Z));
  Jot->C = degrees(acos((LCD*LCD + LBC*LBC - L3*L3)/(2 * LCD * LBC)));
  tempA = degrees(atan((L2 + L4)/L1));
  tempB = degrees(acos((L3*L3+LBC*LBC-LCD*LCD)/(2*L3*LBC)));
  Jot->B = 180.0f - 80.0f - tempA - tempB;

  jotLimit(Jot);
 
  Servo->A = (int)(JA2SA(Jot->A));
  Servo->B = (int)(JB2SB(Jot->B));
  Servo->C = (int)(JC2SC(Jot->C,Jot->B));

  servoLimit(Servo);
}

void RobotArm::servoLimit(Servo_Ang_t *Servo)
{
  if(Servo->A < SERVEA_MIN_ANGLE)
  {
    Servo->A = SERVEA_MIN_ANGLE;
  }
  else if(Servo->A > SERVEA_MAX_ANGLE)
  {
    Servo->A = SERVEA_MAX_ANGLE;
  }

  if(Servo->B < SERVEB_MIN_ANGLE)
  {
    Servo->B = SERVEB_MIN_ANGLE;
  }
  else if(Servo->B > SERVEB_MAX_ANGLE)
  {
    Servo->B = SERVEB_MAX_ANGLE;
  }

  if(Servo->C < SERVEC_MIN_ANGLE)
  {
    Servo->C = SERVEC_MIN_ANGLE;
  }
  else if(Servo->C > SERVEC_MAX_ANGLE)
  {
    Servo->C = SERVEC_MAX_ANGLE;
  }

  if(Servo->D < SERVED_MIN_ANGLE)
  {
    Servo->D = SERVED_MIN_ANGLE;
  }
  else if(Servo->D > SERVED_MAX_ANGLE)
  {
    Servo->D = SERVED_MAX_ANGLE;
  }
}

void RobotArm::jotLimit(Jot_Ang_t *Jot)
{
  if(Jot->A < SA2JA(SERVEA_MIN_ANGLE))
  {
    Jot->A = SA2JA(SERVEA_MIN_ANGLE);
  }
  else if(Jot->A > SA2JA(SERVEA_MAX_ANGLE))
  {
    Jot->A = SA2JA(SERVEA_MAX_ANGLE);
  }

  if(Jot->B < SB2JB(SERVEB_MIN_ANGLE))
  {
    Jot->B = SB2JB(SERVEB_MIN_ANGLE);
  }
  else if(Jot->B > SB2JB(SERVEB_MAX_ANGLE))
  {
    Jot->B = SB2JB(SERVEB_MAX_ANGLE);
  }

  if(Jot->C < SC2JC(SERVEC_MIN_ANGLE,SERVEB_MIN_ANGLE))
  {
    Jot->C = SC2JC(SERVEC_MIN_ANGLE,SERVEB_MIN_ANGLE);
  }
  else if(Jot->C > SC2JC(SERVEC_MAX_ANGLE,SERVEB_MAX_ANGLE))
  {
    Jot->C = SC2JC(SERVEC_MAX_ANGLE,SERVEB_MAX_ANGLE);
  }
}

void RobotArm::servoAngleMove(Servo *Servo,int CurAng,int TarAng)
{
  log_printf("CurAng = %d,TarAng = %d\n",CurAng,TarAng);
  if(CurAng < TarAng)
  {
    while(CurAng != TarAng)
    {
      CurAng++;
      Servo->write(CurAng);
      // log_printf("CurAng = %d\n",CurAng);
      delay(20);
    }
  }
  else if(CurAng > TarAng)
  {
    while(CurAng != TarAng)
    {
      CurAng--;
      Servo->write(CurAng);
      // log_printf("CurAng = %d\n",CurAng);
      delay(20);
    }
  }
}

void RobotArm::jawAngleControl(uint8_t status)
{
  if(status == JAW_CLOSE)
  {
    log_printf("Jaw Angle Closing...\n");
    TarServo.D = JAW_CLOSE_ANGLE;
  }
  else
  {
    log_printf("Jaw Angle Opening...\n");
    TarServo.D = JAW_OPEN_ANGLE;
  }
  
  servoAngleMove(&ServoD,CurServo.D,TarServo.D);
}

void RobotArm::servoAngleControl(uint8_t dir)
{
  if(dir == FORWARD)
  {
    log_printf("ServoA Angle Moving...\n");
    servoAngleMove(&ServoA,CurServo.A,TarServo.A);
    log_printf("ServoB Angle Moving...\n");
    servoAngleMove(&ServoB,CurServo.B,TarServo.B);
    log_printf("ServoC Angle Moving...\n");
    servoAngleMove(&ServoC,CurServo.C,TarServo.C);
  }
  else
  {
    log_printf("ServoC Angle Moving...\n");
    servoAngleMove(&ServoC,CurServo.C,TarServo.C);
    log_printf("ServoB Angle Moving...\n");
    servoAngleMove(&ServoB,CurServo.B,TarServo.B);
    log_printf("ServoA Angle Moving...\n");
    servoAngleMove(&ServoA,CurServo.A,TarServo.A);
  }
}

void RobotArm::servoTest(int id)
{
  switch(id)
  {
    case 0:
      TarServo.A = SERVEA_MAX_ANGLE;
      servoAngleMove(&ServoA,CurServo.A,TarServo.A);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      TarServo.A = SERVEA_INIT_ANGLE;
      servoAngleMove(&ServoA,CurServo.A,TarServo.A);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      break;
     case 1:
      TarServo.B = SERVEB_MAX_ANGLE;
      servoAngleMove(&ServoB,CurServo.B,TarServo.B);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      TarServo.B = SERVEB_INIT_ANGLE;
      servoAngleMove(&ServoB,CurServo.B,TarServo.B);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      break;
    case 2:
      TarServo.C = SERVEC_MAX_ANGLE;
      servoAngleMove(&ServoC,CurServo.C,TarServo.C);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      TarServo.C = SERVEC_INIT_ANGLE;
      servoAngleMove(&ServoC,CurServo.C,TarServo.C);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      break;
    case 3:
      TarServo.D = SERVED_MAX_ANGLE;
      servoAngleMove(&ServoD,CurServo.D,TarServo.D);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      TarServo.D = SERVED_INIT_ANGLE;
      servoAngleMove(&ServoD,CurServo.D,TarServo.D);
      updateServoParams(&LastServo,&CurServo,&TarServo);
      break;
    default:
      break;
  }
}

void RobotArm::forwardKinematicsControl(int A,int B,int C)
{
    //1.获取用户输入的机械臂目标关节角度
    TarJot.A = (float)(A * 1.0f);
    TarJot.B = (float)(B * 1.0f);
    TarJot.C = (float)(C * 1.0f);
    jotLimit(&TarJot);

    //2.根据机械臂目标关节角度计算得到夹爪目标坐标
    forwardKinematics(&TarJaw,&TarJot,&TarServo);

    //3.显示目标的参数和当前的参数
    log_printf("[0]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[1]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);

    log_printf("[2]Target:\n");
    showParams(&TarJaw,&TarJot,&TarServo);

    //4.移动舵机角度到目标角度
    servoAngleControl(FORWARD);

    jawAngleControl(JAW_CLOSE);
  
    //5.更新舵机的当前参数
    updateParams();

    //6.显示舵机当前的参数
    log_printf("[3]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[4]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);
}

void RobotArm::forwardKinematicsControl2(int A,int B,int C)
{
    //1.获取用户输入的机械臂目标关节角度
    TarJot.A = (float)(A * 1.0f);
    TarJot.B = (float)(B * 1.0f);
    TarJot.C = (float)(C * 1.0f);
    jotLimit(&TarJot);

    //2.根据机械臂目标关节角度计算得到夹爪目标坐标
    forwardKinematics(&TarJaw,&TarJot,&TarServo);

    //3.显示目标的参数和当前的参数
    log_printf("[0]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[1]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);

    log_printf("[2]Target:\n");
    showParams(&TarJaw,&TarJot,&TarServo);

    //4.移动舵机角度到目标角度
    servoAngleControl(BACKWARD);

    jawAngleControl(JAW_OPEN);

    //5.更新舵机的当前参数
    updateParams();

    //6.显示舵机当前的参数
    log_printf("[3]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[4]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);
}

void RobotArm::backwardKinematicsControl(int X,int Y,int Z)
{
    //1.获取用户输入的夹爪目标坐标
    TarJaw.X = (float)(X * 1.0f);
    TarJaw.Y = (float)(Y * 1.0f);
    TarJaw.Z = (float)(Z * 1.0f);

    //2.根据夹爪目标坐标计算得到机械臂目标关节角度
    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    //3.显示目标的参数和当前的参数
    log_printf("[0]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[1]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);

    log_printf("[2]Target:\n");
    showParams(&TarJaw,&TarJot,&TarServo);

    //4.移动舵机角度到目标角度
    servoAngleControl(FORWARD);

    jawAngleControl(JAW_CLOSE);

    //5.更新舵机的当前参数
    updateParams();

    //6.显示舵机当前的参数
    log_printf("[3]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[4]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);
}

void RobotArm::backwardKinematicsControl2(int X,int Y,int Z)
{
    //1.获取用户输入的夹爪目标坐标
    TarJaw.X = (float)(X * 1.0f);
    TarJaw.Y = (float)(Y * 1.0f);
    TarJaw.Z = (float)(Z * 1.0f);

    //2.根据夹爪目标坐标计算得到机械臂目标关节角度
    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    //3.显示目标的参数和当前的参数
    log_printf("[0]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[1]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);

    log_printf("[2]Target:\n");
    showParams(&TarJaw,&TarJot,&TarServo);

    //4.移动舵机角度到目标角度
    servoAngleControl(BACKWARD);
  
    jawAngleControl(JAW_OPEN);

    //5.更新舵机的当前参数
    updateParams();

    //6.显示舵机当前的参数
    log_printf("[3]Last:\n");
    showParams(&LastJaw,&LastJot,&LastServo);

    log_printf("[4]Current:\n");
    showParams(&CurJaw,&CurJot,&CurServo);
}

void RobotArm::jawControl(int X,int Y,int Z)
{
    float LX,LY,LZ;
    
    //1.先到目标坐标抓取物品
    TarJaw.X = (float)(X * 1.0f);
    TarJaw.Y = (float)(Y * 1.0f);
    TarJaw.Z = (float)(Z * 1.0f);

    LX = CurJaw.X;
    LY = CurJaw.Y;
    LZ = CurJaw.Z;
    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    servoAngleControl(FORWARD);

    jawAngleControl(JAW_CLOSE);

    updateParams();

    //2.归位
    TarJaw.X = LX;
    TarJaw.Y = LY;
    TarJaw.Z = LZ;

    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    servoAngleControl(BACKWARD);

    updateParams();

    //3.再去往目标坐标对称的方向放下物品
    TarJaw.X = (float)(-X * 1.0f);
    TarJaw.Y = (float)(Y * 1.0f);
    TarJaw.Z = (float)(Z * 1.0f);

    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    servoAngleControl(FORWARD);

    jawAngleControl(JAW_OPEN);

    updateParams();

    //4.归位
    TarJaw.X = LX;
    TarJaw.Y = LY;
    TarJaw.Z = LZ;

    backwardKinematics(&TarJaw,&TarJot,&TarServo);

    servoAngleControl(BACKWARD);

    updateParams();
}