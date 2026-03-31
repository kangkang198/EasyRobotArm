#include <Arduino.h>
#include <ESP32Servo.h>
#ifdef __cplusplus
extern "C" {
#endif
  #include <shell_port.h>
#ifdef __cplusplus
} /* extern "C" */
#endif
#include "robotarm.h"

RobotArm robotarm;

void setup() 
{
  userShellInit();
  robotarm.init();
}

void loop() 
{
}


