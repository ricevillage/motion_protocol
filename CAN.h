#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"

#ifndef CAN_H
#define CAN_H

void Initialize(uint8_t CAN_BUS);
void PrintMessage(uint8_t CAN_BUS, const CAN_Message *message);
void WriteCmd(uint8_t CAN_BUS, const CAN_Message *TxMessage);
void ReadCmd(uint8_t CAN_BUS, CAN_Message *RxMessage);
void Cleanup(uint8_t CAN_BUS);

#endif // CAN_H
