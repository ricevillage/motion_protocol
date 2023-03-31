#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "InverseKinematics.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142

void calibrateMotor(uint16_t id)
{
  WritePositionZeroToRom(id);
  writeEncoderOffset(id, 0);
  readPosition(id);
  readEncoderData(id);
}

int main()
{
  Initialize();

  // writePosition2(MOTOR_ID1, 300, 75);
  // writePosition2(MOTOR_ID2, 300, 104);

  moveLegInZDirection(MOTOR_ID1, MOTOR_ID2, 0.11);

  // while (1)
  // {
  //   readPosition(MOTOR_ID2);
  // }

  Cleanup();
  return 0;
}
