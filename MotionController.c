#include "MotionController.h"

int currentState = INITIAL_STATE;

void setHipPitchStiffness()
{
	uint16_t maxSpeed = 100;
	int32_t angle = 10;

	// front hip pitch
	writePositionAngle(CAN_BUS1, 0x142, maxSpeed, -angle + 100);
	writePositionAngle(CAN_BUS1, 0x145, maxSpeed, angle + 100);

	// back hip pitch
	writePositionAngle(CAN_BUS2, 0x142, maxSpeed, -angle);
	writePositionAngle(CAN_BUS2, 0x145, maxSpeed, angle);
}

void setHipRollStiffness()
{
	uint16_t maxSpeed = 300;
	int32_t angle = 10; // 15000 when motor is fixed;

	// front hip roll
//	writePositionAngle(CAN_BUS1, 0x141, maxSpeed, -angle);
	writePositionAngle(CAN_BUS1, 0x144, maxSpeed, angle);


	// back hip roll
	writePositionAngle(CAN_BUS2, 0x141, maxSpeed, angle);
	writePositionAngle(CAN_BUS2, 0x144, maxSpeed, -angle);
}

void performRobotStandTest()
{
	clearAllMotorStates();

	uint16_t maxSpeed = 100;
	int32_t deltaAngle = 20000;

	setHipPitchStiffness();
	setHipRollStiffness();

	while(1)
	{
		// expand front knees
		writePositionAngle(CAN_BUS1, 0x143, maxSpeed, deltaAngle);
		writePositionAngle(CAN_BUS1, 0x146, maxSpeed, -deltaAngle);

		// expand back knees
		writePositionAngle(CAN_BUS2, 0x143, maxSpeed, deltaAngle);
		writePositionAngle(CAN_BUS2, 0x146, maxSpeed, -deltaAngle);

		sleep(10);
		deltaAngle/=2;

		for(int32_t i = 0; i < 20; i++)
		{
			// contract front knees
			writePositionAngle(CAN_BUS1, 0x143, maxSpeed, -deltaAngle/10);
			writePositionAngle(CAN_BUS1, 0x146, maxSpeed, deltaAngle/10);

			// contract back knees
			writePositionAngle(CAN_BUS2, 0x143, maxSpeed, -deltaAngle/10);
			writePositionAngle(CAN_BUS2, 0x146, maxSpeed, deltaAngle/10);
		}
	}

	clearAllMotorStates();
}

void robotStandUpCommand()
{
	clearAllMotorStates();

	uint16_t maxSpeed = 100;
	int32_t deltaAngle = 20000;

	setHipPitchStiffness();
	setHipRollStiffness();

	// expand front knees
	writePositionAngle(CAN_BUS1, 0x143, maxSpeed, deltaAngle);
	writePositionAngle(CAN_BUS1, 0x146, maxSpeed, -deltaAngle);

	// expand back knees
	writePositionAngle(CAN_BUS2, 0x143, maxSpeed, deltaAngle);
	writePositionAngle(CAN_BUS2, 0x146, maxSpeed, -deltaAngle);

//	clearAllMotorStates();
}

void robotSitDownCommand()
{
	clearAllMotorStates();

	uint16_t maxSpeed = 100;
	int32_t deltaAngle = 20000;

	deltaAngle/=2;

	for(int32_t i = 0; i < 20; i++)
	{
		// contract front knees
		writePositionAngle(CAN_BUS1, 0x143, maxSpeed, -deltaAngle/10);
		writePositionAngle(CAN_BUS1, 0x146, maxSpeed, deltaAngle/10);

		// contract back knees
		writePositionAngle(CAN_BUS2, 0x143, maxSpeed, -deltaAngle/10);
		writePositionAngle(CAN_BUS2, 0x146, maxSpeed, deltaAngle/10);
	}

//	clearAllMotorStates();
}

/*
 left side from robot POV: CAN_BUS1/CAN_BUS2: 0x146, 0x145
 right side from robot POV: CAN_BUS1/CAN_BUS2: 0x143, 0x142
 */
void legGaitForward(int file_desc)
{
	clearAllMotorStates();
	setHipRollStiffness();
	currentState = INITIAL_STATE;

	while(currentState == INITIAL_STATE)
	{
	    // knee expand
	    writePositionAngle(CAN_BUS1, 0x146, 100, -40000);
	    writePositionAngle(CAN_BUS2, 0x146, 100, -60000);

	    writePositionAngle(CAN_BUS1, 0x143, 100, 40000);
	    writePositionAngle(CAN_BUS2, 0x143, 100, 40000);
	    sleep(3);
	    // hip up
	    writePositionAngle(CAN_BUS1, 0x145, 200, -20000);
	    writePositionAngle(CAN_BUS2, 0x145, 200, -30000);

	    writePositionAngle(CAN_BUS1, 0x142, 200, 20000);
	    writePositionAngle(CAN_BUS2, 0x142, 200, 20000);
	    // knee contract
	    writePositionAngle(CAN_BUS1, 0x146, 300, 40000);
	    writePositionAngle(CAN_BUS2, 0x146, 300, 60000);

	    writePositionAngle(CAN_BUS1, 0x143, 300, -40000);
	    writePositionAngle(CAN_BUS2, 0x143, 300, -40000);
	    sleep(1);
	    // hip down
	    writePositionAngle(CAN_BUS1, 0x145, 300, 20000);
	    writePositionAngle(CAN_BUS2, 0x145, 300, 30000);

	    writePositionAngle(CAN_BUS1, 0x142, 300, -20000);
	    writePositionAngle(CAN_BUS2, 0x142, 300, -20000);

		updateChannelState(file_desc);
	}
}

void legGaitBackward(int file_desc)
{
	clearAllMotorStates();
	setHipRollStiffness();
	currentState = INITIAL_STATE;

	while(currentState == INITIAL_STATE)
	{
		// knee contract
		writePositionAngle(CAN_BUS1, 0x146, 100, 20000);
		writePositionAngle(CAN_BUS2, 0x146, 300, 30000);

	    writePositionAngle(CAN_BUS1, 0x143, 300, -20000);
	    writePositionAngle(CAN_BUS2, 0x143, 300, -20000);
		sleep(3);
	    // hip up
	    writePositionAngle(CAN_BUS1, 0x145, 200, -20000);
	    writePositionAngle(CAN_BUS2, 0x145, 200, -30000);

	    writePositionAngle(CAN_BUS1, 0x142, 200, 20000);
	    writePositionAngle(CAN_BUS2, 0x142, 200, 20000);
		// knee expand
		writePositionAngle(CAN_BUS1, 0x146, 300, -20000);
		writePositionAngle(CAN_BUS2, 0x146, 300, -30000);

	    writePositionAngle(CAN_BUS1, 0x143, 100, 20000);
	    writePositionAngle(CAN_BUS2, 0x143, 100, 20000);
		sleep(1);
	    // hip down
	    writePositionAngle(CAN_BUS1, 0x145, 300, 20000);
	    writePositionAngle(CAN_BUS2, 0x145, 300, 30000);

	    writePositionAngle(CAN_BUS1, 0x142, 300, -20000);
	    writePositionAngle(CAN_BUS2, 0x142, 300, -20000);

		updateChannelState(file_desc);
	}
}

void clearAllMotorStates()
{
	// front legs
//	clearState(CAN_BUS1, 0x141);
	clearState(CAN_BUS1, 0x142);
	clearState(CAN_BUS1, 0x143);

	clearState(CAN_BUS1, 0x144);
	clearState(CAN_BUS1, 0x145);
	clearState(CAN_BUS1, 0x146);

	// back legs
	clearState(CAN_BUS2, 0x141);
	clearState(CAN_BUS2, 0x142);
	clearState(CAN_BUS2, 0x143);

	clearState(CAN_BUS2, 0x144);
	clearState(CAN_BUS2, 0x145);
	clearState(CAN_BUS2, 0x146);
}

void StopAllMotors()
{
	// front legs
//	motorPause(CAN_BUS1, 0x141);
	motorPause(CAN_BUS1, 0x142);
	motorPause(CAN_BUS1, 0x143);

	motorPause(CAN_BUS1, 0x144);
	motorPause(CAN_BUS1, 0x145);
	motorPause(CAN_BUS1, 0x146);

	// back legs
	motorPause(CAN_BUS2, 0x141);
	motorPause(CAN_BUS2, 0x142);
	motorPause(CAN_BUS2, 0x143);

	motorPause(CAN_BUS2, 0x144);
	motorPause(CAN_BUS2, 0x145);
	motorPause(CAN_BUS2, 0x146);
}

void printJoingAngles(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollId)
{
	while (1)
	{
		int32_t kneeAngle = readPosition(CAN_BUS, kneeId);  // Knee
		int32_t hipAngle = readPosition(CAN_BUS, hipId); // Hip
		int32_t hipRollAngle = readPosition(CAN_BUS, hipRollId); // Hip Roll

		printf("Knee Position: %d\n", kneeAngle);
		printf("Hip Position: %d\n", hipAngle);
		printf("Hip Roll Position: %d\n", hipRollAngle);
	}
}

void calibrateMotor(uint8_t CAN_BUS, uint16_t id)
{
	WritePositionZeroToRom(CAN_BUS, id);
	writeEncoderOffset(CAN_BUS, id, 0);
	readPosition(CAN_BUS, id);
	readEncoderData(CAN_BUS, id);
}


