#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "MotionController.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142
#define MOTOR_ID3 0x143

/*
 * Project github: https://github.com/ricevillage/motion_protocol
 */

void calibrateMotor(uint8_t CAN_BUS, uint16_t id)
{
  WritePositionZeroToRom(CAN_BUS, id);
  writeEncoderOffset(CAN_BUS, id, 0);
  readPosition(CAN_BUS, id);
  readEncoderData(CAN_BUS, id);
}

int main()
{
  Initialize(CAN_BUS1);

  printJoingAngles(CAN_BUS1, MOTOR_ID3, MOTOR_ID2, MOTOR_ID1);

//  while(1)
//  {
//	  readEncoderData(CAN_BUS1, 0x141);
//	  readPosition(CAN_BUS1, 0x141);
//  }

  Cleanup(CAN_BUS1);
  return 0;
}
