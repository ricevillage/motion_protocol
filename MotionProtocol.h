
#ifndef MotionProtocol_H
#define MotionProtocol_H

#include <stdint.h>
#include "LoopBack.c"

#define SRR 1
#define DLC 8

int8_t temperature;
uint8_t posKp, posKi, velKp, velKi, curKp, curKi;
uint16_t motorPower;
int16_t current, velocity, motorShaftAngle;
int32_t motorAngle, acceleration;

void setCommonFields(CAN_Message *message, uint16_t id);

// Commands

// Reading data
void readPID(uint16_t id);
void readAcceleration(uint16_t id);
void readPosition(uint16_t id);
void readPower(uint16_t id);

// Writing data
void writePID(uint16_t id, uint8_t currentPidKp, uint8_t currentPidKi,
              uint8_t speedPidKp, uint8_t speedPidKi,
              uint8_t positionPidKp, uint8_t positionPidKi);
void writeAcceleration(uint16_t id, uint32_t acceleration);
void clearState(uint16_t id);
void writeTorqueCurrent(uint16_t id, int16_t iqControl);
void writeVelocity(uint16_t id, int32_t speedControl);
void writePosition(uint16_t id, uint16_t maxSpeed, int32_t angleControl);

#endif // MotionProtocol_H