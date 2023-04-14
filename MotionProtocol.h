
#ifndef MotionProtocol_H
#define MotionProtocol_H

#include <stdint.h>
#include "PmodCAN.h"
#include "CAN.h"

#define DLC 8
#define C 0
#define CC 1

void setCommonFields(CAN_Message *message, uint16_t id);

// PID control
void readPidData(uint8_t CAN_BUS, uint16_t id);
void writePidToRam(uint8_t CAN_BUS, uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);
void writePidToRom(uint8_t CAN_BUS, uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);

// Motor Control
void clearState(uint8_t CAN_BUS, uint16_t id);
void motorPause(uint8_t CAN_BUS, uint16_t id);
void motorResume(uint8_t CAN_BUS, uint16_t id);
void writeTorqueCurrent(uint8_t CAN_BUS, uint16_t id, int8_t iqControlAmp);
void writeVelocity(uint8_t CAN_BUS, uint16_t id, uint16_t speedControl);
void writePosition1(uint8_t CAN_BUS, uint16_t id, int16_t angleControlDegree);
void writePosition2(uint8_t CAN_BUS, uint16_t id, uint16_t maxSpeed, int16_t angleControlDegree);
void writePosition3(uint8_t CAN_BUS, uint16_t id, uint8_t spinDirection, uint16_t angleControlDegree);
void writePosition4(uint8_t CAN_BUS, uint16_t id, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControlDegree);

// Encoder and Position Control
int32_t readAccelerationData(uint8_t CAN_BUS, uint16_t id);
void writeAccelerationToRam(uint8_t CAN_BUS, uint16_t id, int16_t inputAcceleration_RPSS);
void readEncoderData(uint8_t CAN_BUS, uint16_t id);
void writeEncoderOffset(uint8_t CAN_BUS, uint16_t id, uint16_t inputEncoderOffset);
void WritePositionZeroToRom(uint8_t CAN_BUS, uint16_t id);
int32_t readPosition(uint8_t CAN_BUS, uint16_t id);
uint16_t readCircleAngle(uint8_t CAN_BUS, uint16_t id);

// Motor Status
void readMotorStatus1(uint8_t CAN_BUS, uint16_t id);
void clearErrorFlag(uint8_t CAN_BUS, uint16_t id);
void readMotorStatus2(uint8_t CAN_BUS, uint16_t id);

#endif // MotionProtocol_H
