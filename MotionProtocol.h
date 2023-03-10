
#ifndef MotionProtocol_H
#define MotionProtocol_H

#include <stdint.h>
#include "PmodCAN.h"
#include "CAN.h"

#define DLC 8

void setCommonFields(CAN_Message *message, uint16_t id);

// Commands

// Reading data
void readPID(uint16_t id);
int32_t readAcceleration(uint16_t id);
int32_t readPosition(uint16_t id);

// Writing data
void writePID(uint16_t id, uint8_t currentPidKp, uint8_t currentPidKi,
              uint8_t speedPidKp, uint8_t speedPidKi,
              uint8_t positionPidKp, uint8_t positionPidKi);
void writeAcceleration(uint16_t id, uint32_t acceleration);
void clearState(uint16_t id);
void writeTorqueCurrent(uint16_t id, int16_t iqControl);
void writeVelocity(uint16_t id, int32_t speedControl);
void writePosition(uint16_t id, uint16_t maxSpeed, int32_t angleControl);

// Getters
int8_t get_temperature();
uint8_t get_posKp();
uint8_t get_posKi();
uint8_t get_velKp();
uint8_t get_velKi();
uint8_t get_curKp();
uint8_t get_curKi();
uint16_t get_motorPower();
int16_t get_current();
int16_t get_velocity();
int16_t get_motorShaftAngle();
int32_t get_motorAngle();
int32_t get_acceleration();


#endif // MotionProtocol_H
