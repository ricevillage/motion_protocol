#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "InverseKinematics.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142

void moveKneeUp(uint16_t motorId1, uint16_t motorId2, double kneeHeight)
{
  Initialize();
  clearState(motorId1);
  clearState(motorId2);
  calculateLegJointAngles(motorId1, motorId2, kneeHeight);
  clearState(motorId1);
  clearState(motorId2);
  Cleanup();
}

int main()
{
  double kneeHeight = 0.11;
  moveKneeUp(motorId1, motorId2, kneeHeight);
  return 0;
}
