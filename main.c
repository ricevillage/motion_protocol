#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "MotionController.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142
#define MOTOR_ID3 0x143

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

//  calibrateMotor(MOTOR_ID3);
//  calibrateMotor(MOTOR_ID2);

//  printJoingAngles(MOTOR_ID3, MOTOR_ID2, MOTOR_ID1);

//  legLoop1(MOTOR_ID3, MOTOR_ID2);


  while(1)
  {
	  readEncoderData(0x144);
  }


  Cleanup();
  return 0;
}
