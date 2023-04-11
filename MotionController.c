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

	   printf("Knee Position: %d\n", kneeAngle);
	   printf("Hip Position: %d\n", hipAngle);
   }
}

void legLoop1(uint16_t kneeId, uint16_t hipId)
{

	clearState(kneeId);
	clearState(hipId);


	int32_t deltaTheta = 25000;
	uint16_t hipSpeed = 300;

	int32_t initialKneePosition = readPosition(kneeId);
	int32_t initialHipPosition = readPosition(hipId);


	writePosition4(kneeId, 0, 300, initialKneePosition + 20000);

	sleep(1);

	while(1)
	{
		writePosition2(hipId, hipSpeed, initialHipPosition + deltaTheta);
		writePosition2(hipId, hipSpeed, initialHipPosition);
	}


	clearState(kneeId);
	clearState(hipId);
}

void legLoop2(uint16_t kneeId, uint16_t hipId)
{

}
