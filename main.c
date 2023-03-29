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

//  writePosition2(MOTOR_ID1, 200, 0);
//  writePosition2(MOTOR_ID2, 200, 0);
//
//  WritePositionZeroToRom(MOTOR_ID1);
//  WritePositionZeroToRom(MOTOR_ID2);
//
//  writeEncoderOffset(MOTOR_ID1, 0);
//  writeEncoderOffset(MOTOR_ID2, 0);

    writePosition2(MOTOR_ID1, 300, 75);
    writePosition2(MOTOR_ID2, 300, 104);

    moveKneeToDesiredHeight(MOTOR_ID1, MOTOR_ID2, 0.11);

//    readPosition(MOTOR_ID1);
//    readEncoderData(MOTOR_ID1);
//
//    readPosition(MOTOR_ID2);
//    readEncoderData(MOTOR_ID2);

//    while(1) {
//    	readPosition(MOTOR_ID2);
//    }

  Cleanup();
  return 0;
}
