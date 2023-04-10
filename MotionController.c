#include "MotionController.h"

//void test3DOF(uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId)
//{
//   readPosition(MOTOR_ID1); // Hip Roll
//   readPosition(MOTOR_ID3); // Knee
//   readPosition(MOTOR_ID2); // Hip
//
//   // knee
//	writePosition2(MOTOR_ID3, 300, 40);
//	writePosition2(MOTOR_ID3, 300, 20);
//
//   // hip
//	writePosition2(MOTOR_ID2, 100, 20);
//	writePosition2(MOTOR_ID2, 100, 0);
//
//
//	// hip roll
//	writePosition2(MOTOR_ID1, 100, 10);
//	writePosition2(MOTOR_ID1, 100, 40);
//}

void printJoingAngles(uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId)
{
   while (1)
   {
	   int32_t kneeAngle = readPosition(kneeId);  // Knee
	   int32_t hipAngle = readPosition(hipId); // Hip

	   printf("Knee Angle: %d\n", kneeAngle);
	   printf("Hip Angle: %d\n", hipAngle);
   }
}

void legLoop1(uint16_t kneeId, uint16_t hipId)
{
	int32_t initialHipAngle = 5;
	int32_t initialkneeAngle = 70;

	int32_t gaitAngle = 5;
	int32_t deltaTheta = 50;
	uint16_t hipSpeed = 700;

	writePosition2(hipId, hipSpeed, initialHipAngle);
	writePosition2(kneeId, hipSpeed, initialkneeAngle);

	while(1)
	{
		writePosition2(hipId, hipSpeed, gaitAngle);
		writePosition2(hipId, hipSpeed, gaitAngle + deltaTheta);
	}

	writePosition2(hipId, hipSpeed, initialHipAngle);
	writePosition2(kneeId, hipSpeed, initialkneeAngle);
}

void legLoop2(uint16_t kneeId, uint16_t hipId)
{
	int32_t initialHipAngle = 5;
	int32_t initialkneeAngle = 70;

	int32_t gaitAngle = 65;
	int32_t deltaTheta = 50;
	uint16_t hipSpeed = 700;

	writePosition2(hipId, hipSpeed, initialHipAngle);
	writePosition2(kneeId, hipSpeed, initialkneeAngle);

	while(1)
	{
		writePosition2(hipId, hipSpeed, gaitAngle);
		writePosition2(hipId, hipSpeed, gaitAngle - deltaTheta);
	}

	writePosition2(hipId, hipSpeed, initialHipAngle);
	writePosition2(kneeId, hipSpeed, initialkneeAngle);
}
