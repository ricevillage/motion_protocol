
#ifndef MotionProtocol_H
#define MotionProtocol_H

#include <stdint.h>
#include "PmodCAN.h"
#include "CAN.h"

#define DLC 8
#define POSITION_FACTOR 580

void setCommonFields(CAN_Message *message, uint16_t id);

// PID control
void readPidData(uint16_t id);
void writePidToRam(uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);
void writePidToRom(uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi);

// Motor Control
void clearState(uint16_t id);
void motorPause(uint16_t id);
void motorResume(uint16_t id);
void writeTorqueCurrent(uint16_t id, int8_t iqControlAmp);
void writeVelocity(uint16_t id, uint16_t speedControl);
void writePosition1(uint16_t id, int16_t angleControlDegree);
void writePosition2(uint16_t id, uint16_t maxSpeed, int16_t angleControlDegree);
void writePosition3(uint16_t id, uint8_t spinDirection, uint16_t angleControlDegree);
void writePosition4(uint16_t id, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControlDegree);

// Encoder and Position Control
int32_t readAccelerationData(uint16_t id);
void writeAccelerationToRam(uint16_t id, int16_t inputAcceleration_RPSS);
void readEncoderData(uint16_t id);
void writeEncoderOffset(uint16_t id, uint16_t inputEncoderOffset);
void WritePositionZeroToRom(uint16_t id);
int32_t readPosition(uint16_t id);
uint8_t readCircleAngle(uint16_t id);

// Motor Status
void readMotorStatus1(uint16_t id);
void clearErrorFlag(uint16_t id);
void readMotorStatus2(uint16_t id);

#endif // MotionProtocol_H
