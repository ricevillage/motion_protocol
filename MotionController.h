
#ifndef MotionController_H
#define MotionController_H

#include <stdint.h>
#include "MotionProtocol.h"

void test3DOF(uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId);
void printJoingAngles(uint16_t kneeId, uint16_t hipId, uint16_t hipRollkneeId);
void legLoop1(uint16_t kneeId, uint16_t hipId);
void legLoop2(uint16_t kneeId, uint16_t hipId);

#endif // MotionController_H
