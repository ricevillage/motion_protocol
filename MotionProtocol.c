
#include "MotionProtocol.h"

int8_t temperature;
uint8_t posKp, posKi, velKp, velKi, curKp, curKi;
uint16_t motorPower;
int16_t current, velocity, motorShaftAngle;
int32_t motorAngle, acceleration;


void setCommonFields(CAN_Message *message, uint16_t id)
{
    message->id = id;   // 11 bit id
    message->dlc = DLC; // Data length
    message->eid = 0x0;
    message->rtr = 0;
    message->ide = 0;
}

void readPID(uint16_t id)
{
    // Create a CAN message for transmission and reception
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    // Set common fields of TxMessage
    setCommonFields(&TxMessage, id);

    // Set message ID and data bytes if needed
    TxMessage.data[0] = 0x30; // Get PID constants

    WriteCmd(&TxMessage); // Send the message
    sleep(1);            // Wait for the motor to respond
    ReadCmd(&RxMessage);  // Read the response

    // Parse the response and update the PID values
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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    motorPower = ((uint16_t)RxMessage.data[7] << 8) |
                 RxMessage.data[6];
}

/*
    This writes current, speed, position loop KP, and KI parameters to RAM, but they are not saved after power off. The maximum range of PI parameters depends on the motor model. Users need to adjust only 0-256 units.

    For example:
    - Max current loop = 3 (set by the system)
    - Input = 85
    - 1 unit = 3/256 = 0.01171875
    - Current loop KP = 85* 0.01171875 = 0.99609375
*/

void writePID(uint16_t id, uint8_t currentPidKp, uint8_t currentPidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t positionPidKp, uint8_t positionPidKi)
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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    curKp = RxMessage.data[2];
    curKi = RxMessage.data[3];
    velKp = RxMessage.data[4];
    velKi = RxMessage.data[5];
    posKp = RxMessage.data[6];
    posKi = RxMessage.data[7];
}

/*
    This writes acceleration to RAM, but it is not saved after power off. The parameter range is between 50-80000.

    For example:
    - Actual acceleration = acceleration * 1dps/s
    - Acceleration = 10000 => 10000*1 = 10000dps/s
    - 1 dps/s = 0.0174533 rad/s^2
    - 10000 dps/s * 0.0174533 (rad/s^2 / dps/s) = 174.533 rad/s^2
*/

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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    acceleration = ((uint32_t)RxMessage.data[7] << 24) |
                   ((uint32_t)RxMessage.data[6] << 16) |
                   ((uint32_t)RxMessage.data[5] << 8) |
                   RxMessage.data[4];
}

void clearState(uint16_t id)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x80;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);
}

/*
    This writes control commands for the torque and current output.

    For example:
    - Actual torque current = iqControl * 0.01dps/LSB
    - iqControl = 100 => 100*0.01 = 1A
*/

void writeTorqueCurrent(uint16_t id, int16_t iqControl)
{
    CAN_Message TxMessage;
    CAN_Message RxMessage;

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA1;
    TxMessage.data[4] = iqControl & 0xFF;
    TxMessage.data[5] = (iqControl >> 8) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];
}

/*
    This writes control commands for the speed of the motor output shaft.

    For example:
    - Actual speed = speedControl * 0.01dps/LSB
    - speedControl = 10000 => 10000*0.01 = 100dps
    - 1 rad/s = 57.2958 dps
    - 100 dps * (1 rad/s / 57.2958 dps) = 1.74533 rad/s
*/

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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];
}

/*
    This writes control commands for the position of the motor. The rotation direction is equal to the target position minus the current position. MaxSpeed limits the maximum speed of the motor output shaft rotation.

    For example:
    - Actual position = angleControl * 0.01degree/LSB
    - angleControl = 36000 => 36000*0.01 = 360 degrees
    - Actual speed = maxSpeed * 1dps/LSB
    - maxSpeed = 500 => 500*1 = 500dps
*/

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
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[0];
    current = ((uint16_t)RxMessage.data[3] << 8) |
              RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    motorShaftAngle = ((uint16_t)RxMessage.data[7] << 8) |
                      RxMessage.data[6];

}
