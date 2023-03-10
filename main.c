#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "InverseKinematics.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142

int main()
{
  Initialize();
  clearState(MOTOR_ID1);
  clearState(MOTOR_ID2);

  double kneeHeight = 0.11;
  moveKneeToDesiredHeight(MOTOR_ID1, MOTOR_ID2, kneeHeight);

  clearState(MOTOR_ID1);
  clearState(MOTOR_ID2);
  Cleanup();
  return 0;
}
