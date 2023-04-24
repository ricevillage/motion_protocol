#ifndef MotionController_H
#define MotionController_H

#include <stdint.h>
#include "MotionProtocol.h"

enum State
{
	INITIAL_STATE = 0x3670367,
	STANDUP_STATE = 0x3670378,
	SITDOWN_STATE = 0x3780367,
	FORWARD_GAIT_STATE = 0x36703A7,
//	BACKWARD_GAIT_STATE = 0x3A70367,
	STOP_STATE = 0x4780367,
};

void setHipPitchStiffness();
void setHipRollStiffness();
void performRobotStandTest();
void robotStandUpCommand();
void robotSitDownCommand();
void legGaitForward(int file_desc);
void legGaitBackward(int file_desc);
void clearAllMotorStates();
void StopAllMotors();
void printJoingAngles(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollId);
void calibrateMotor(uint8_t CAN_BUS, uint16_t id);

#endif // MotionController_H
