
#include "MotionProtocol.h"

void setCommonFields(CAN_Message *message, uint16_t id)
{
    message->id = id;
    message->ide = IDE;
    message->rtr = RTR;
    message->srr = SRR;
    message->dlc = DLC;
}

void readPID(uint16_t id)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x30;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    curKp = RxMessage.data[2];
    curKi = RxMessage.data[3];
    velKp = RxMessage.data[4];
    velKi = RxMessage.data[5];
    posKp = RxMessage.data[6];
    posKi = RxMessage.data[7];
}

void readAcceleration(uint16_t id)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x42;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    acceleration = ((uint32_t)RxMessage.data[7] << 24) |
                   ((uint32_t)RxMessage.data[6] << 16) |
                   ((uint32_t)RxMessage.data[5] << 8) |
                   RxMessage.data[4];
}

void readPosition(uint16_t id)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x92;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    motorAngle = ((uint32_t)RxMessage.data[7] << 24) |
                 ((uint32_t)RxMessage.data[6] << 16) |
                 ((uint32_t)RxMessage.data[5] << 8) |
                 RxMessage.data[4];
}

void readPower(uint16_t id)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x71;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    motorPower = ((uint16_t)RxMessage.data[7] << 8) |
                 RxMessage.data[6];
}

void writePID(uint16_t id, uint8_t currentPidKp, uint8_t currentPidKi,
              uint8_t speedPidKp, uint8_t speedPidKi,
              uint8_t positionPidKp, uint8_t positionPidKi)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x31;
    TxMessage.data[2] = currentPidKp;
    TxMessage.data[3] = currentPidKi;
    TxMessage.data[4] = speedPidKp;
    TxMessage.data[5] = speedPidKi;
    TxMessage.data[6] = positionPidKp;
    TxMessage.data[7] = positionPidKi;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    curKp = RxMessage.data[2];
    curKi = RxMessage.data[3];
    velKp = RxMessage.data[4];
    velKi = RxMessage.data[5];
    posKp = RxMessage.data[6];
    posKi = RxMessage.data[7];
}

void writeAcceleration(uint16_t id, uint32_t acceleration)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x43;
    TxMessage.data[4] = acceleration & 0xFF;
    TxMessage.data[5] = (acceleration >> 8) & 0xFF;
    TxMessage.data[6] = (acceleration >> 16) & 0xFF;
    TxMessage.data[7] = (acceleration >> 24) & 0xFF;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    acceleration = ((uint32_t)RxMessage.data[7] << 24) |
                   ((uint32_t)RxMessage.data[6] << 16) |
                   ((uint32_t)RxMessage.data[5] << 8) |
                   RxMessage.data[4];
}

void clearState(uint16_t id)
{
    CAN_Message message;
    setCommonFields(&message, id);
    message.data[0] = 0x80;
}

void writeTorqueCurrent(uint16_t id, int16_t iqControl)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA1;
    TxMessage.data[4] = iqControl & 0xFF;
    TxMessage.data[5] = (iqControl >> 8) & 0xFF;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];
}

void writeVelocity(uint16_t id, int32_t speedControl)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA2;
    TxMessage.data[4] = speedControl & 0xFF;
    TxMessage.data[5] = (speedControl >> 8) & 0xFF;
    TxMessage.data[6] = (speedControl >> 16) & 0xFF;
    TxMessage.data[7] = (speedControl >> 24) & 0xFF;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];
}

void writePosition(uint16_t id, uint16_t maxSpeed,
                   int32_t angleControl)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA4;
    TxMessage.data[2] = maxSpeed & 0xFF;
    TxMessage.data[3] = (maxSpeed >> 8) & 0xFF;
    TxMessage.data[4] = angleControl & 0xFF;
    TxMessage.data[5] = (angleControl >> 8) & 0xFF;
    TxMessage.data[6] = (angleControl >> 16) & 0xFF;
    TxMessage.data[7] = (angleControl >> 24) & 0xFF;
    WriteCmd(TxMessage);

    sleep(1);
    ReadCmd(RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];
}
