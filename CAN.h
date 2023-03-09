#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"

#ifndef CAN_H
#define CAN_H

void Initialize();
void PrintMessage(const CAN_Message *message);
void WriteCmd(CAN_Message *TxMessage);
void ReadCmd(CAN_Message *RxMessage);
void Cleanup();

#endif // CAN_H

