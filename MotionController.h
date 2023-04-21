#ifndef MotionController_H
#define MotionController_H

#include <stdint.h>
#include "MotionProtocol.h"

void setHipPitchStiffness();
void setHipRollStiffness();
void performRobotStandTest();
void clearAllMotorStates();
void printJoingAngles(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollId);
void calibrateMotor(uint8_t CAN_BUS, uint16_t id);

#endif // MotionController_H
