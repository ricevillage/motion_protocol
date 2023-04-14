
#ifndef MotionController_H
#define MotionController_H

#include <stdint.h>
#include "MotionProtocol.h"

void test3DOF(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId);
void printJoingAngles(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId);
void legLoop1(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId);
void legLoop2(uint8_t CAN_BUS, uint16_t kneeId, uint16_t hipId);

#endif // MotionController_H
