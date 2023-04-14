#include "MotionController.h"

//void test3DOF(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId)
//{
//   readPosition(CAN_BUS, MOTOR_ID1); // Hip Roll
//   readPosition(CAN_BUS, MOTOR_ID3); // Knee
//   readPosition(CAN_BUS, MOTOR_ID2); // Hip
//
//   // knee
//	writePosition2(CAN_BUS, MOTOR_ID3, 300, 40);
//	writePosition2(CAN_BUS, MOTOR_ID3, 300, 20);
//
//   // hip
//	writePosition2(CAN_BUS, MOTOR_ID2, 100, 20);
//	writePosition2(CAN_BUS, MOTOR_ID2, 100, 0);
//
//
//	// hip roll
//	writePosition2(CAN_BUS, MOTOR_ID1, 100, 10);
//	writePosition2(CAN_BUS, MOTOR_ID1, 100, 40);
//}

void printJoingAngles(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId)
{
   while (1)
   {
	   int32_t kneeAngle = readPosition(CAN_BUS, kneeId);  // Knee
	   int32_t hipAngle = readPosition(CAN_BUS, hipId); // Hip

	   printf("Knee Position: %d\n", kneeAngle);
	   printf("Hip Position: %d\n", hipAngle);
   }
}

void legLoop1(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId)
{

	clearState(CAN_BUS, kneeId);
	clearState(CAN_BUS, hipId);

	int32_t deltaTheta = 30000;
	uint16_t hipSpeed = 300;

	int32_t initialKneePosition = readPosition(CAN_BUS, kneeId);
	int32_t initialHipPosition = readPosition(CAN_BUS, hipId);

	writePosition4(CAN_BUS, kneeId, 0, 300, initialKneePosition + 20000);
	sleep(1);

	while(1)
	{
		// move hip backwards
		writePosition2(CAN_BUS, hipId, hipSpeed, initialHipPosition + deltaTheta);
		// move hip to original position
		writePosition2(CAN_BUS, hipId, hipSpeed, initialHipPosition);
	}


	clearState(CAN_BUS, kneeId);
	clearState(CAN_BUS, hipId);
}

void legLoop2(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId)
{

}
