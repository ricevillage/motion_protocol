#include "MotionProtocol.h"

int8_t temperature;
uint8_t anglePidKp, anglePidKi, speedPidKp, speedPidKi, iqPidKp, iqPidKi;
uint8_t circleAngle, errorState;
int16_t torqueCurrent, velocity, voltage;
int32_t acceleration;
int64_t motorAngle;
uint16_t encoderCurrentPosition, encoderOriginalPosition, encoderOffset;

void setCommonFields(CAN_Message *message, uint16_t id)
{
    message->id = id;   // 11 bit id
    message->dlc = DLC; // Data length
    message->eid = 0x0;
    message->rtr = 0;
    message->ide = 0;
}

// Read the motor's current PID parameters.
void readPidData(uint16_t id)
{
    // Create a CAN message for transmission and reception
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    // Set common fields of TxMessage
    setCommonFields(&TxMessage, id);

    // Set message ID and data bytes if needed
    TxMessage.data[0] = 0x30; // Get PID constants

    WriteCmd(&TxMessage); // Send the message
    sleep(1);             // Wait for the motor to respond
    ReadCmd(&RxMessage);  // Read the response

    // Parse the response and update the PID values
    anglePidKp = RxMessage.data[2];
    anglePidKi = RxMessage.data[3];
    speedPidKp = RxMessage.data[4];
    speedPidKi = RxMessage.data[5];
    iqPidKp = RxMessage.data[6];
    iqPidKi = RxMessage.data[7];
}

/*
    This writes current, speed, position loop KP, and KI parameters to RAM, but they are not saved after power off. The maximum range of PI parameters depends on the motor model. Users need to adjust only 0-256 units.
    For example:
    - Max current loop = 3 (set by the system)
    - Input = 85
    - 1 unit = 3/256 = 0.01171875
    - Current loop KP = 85* 0.01171875 = 0.99609375
*/

void writePidToRam(uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x31;
    TxMessage.data[2] = anglePidKp;
    TxMessage.data[3] = anglePidKi;
    TxMessage.data[4] = speedPidKp;
    TxMessage.data[5] = speedPidKi;
    TxMessage.data[6] = iqPidKp;
    TxMessage.data[7] = iqPidKi;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    anglePidKp = RxMessage.data[2];
    anglePidKi = RxMessage.data[3];
    speedPidKp = RxMessage.data[4];
    speedPidKi = RxMessage.data[5];
    iqPidKp = RxMessage.data[6];
    iqPidKi = RxMessage.data[7];
}

// Write PID parameters to the ROM.
void writePidToRom(uint16_t id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x32;
    TxMessage.data[2] = anglePidKp;
    TxMessage.data[3] = anglePidKi;
    TxMessage.data[4] = speedPidKp;
    TxMessage.data[5] = speedPidKi;
    TxMessage.data[6] = iqPidKp;
    TxMessage.data[7] = iqPidKi;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    anglePidKp = RxMessage.data[2];
    anglePidKi = RxMessage.data[3];
    speedPidKp = RxMessage.data[4];
    speedPidKi = RxMessage.data[5];
    iqPidKp = RxMessage.data[6];
    iqPidKi = RxMessage.data[7];
}

// Read the motor's acceleration data.
int32_t readAccelerationData(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x33;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    acceleration = ((uint32_t)RxMessage.data[7] << 24) |
                   ((uint32_t)RxMessage.data[6] << 16) |
                   ((uint32_t)RxMessage.data[5] << 8) |
                   RxMessage.data[4];
    return acceleration;
}

/*
    This writes acceleration to RAM, but it is not saved after power off.
    inputAcceleration_RPSS [rad/s^2]
*/

void writeAccelerationToRam(uint16_t id, int16_t inputAcceleration_RPSS)
{
    int32_t inputAcceleration = inputAcceleration_RPSS * (180 / M_PI);

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x34;
    TxMessage.data[4] = inputAcceleration & 0xFF;
    TxMessage.data[5] = (inputAcceleration >> 8) & 0xFF;
    TxMessage.data[6] = (inputAcceleration >> 16) & 0xFF;
    TxMessage.data[7] = (inputAcceleration >> 24) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    acceleration = ((uint32_t)RxMessage.data[7] << 24) |
                   ((uint32_t)RxMessage.data[6] << 16) |
                   ((uint32_t)RxMessage.data[5] << 8) |
                   RxMessage.data[4];

    printf("Acceleration: %d\n", acceleration);
}

// Read the current position of the encoder.
void readEncoderData(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x90;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    encoderCurrentPosition = ((uint16_t)RxMessage.data[3] << 8) |
                             RxMessage.data[2];

    encoderOriginalPosition = ((uint16_t)RxMessage.data[5] << 8) |
                              RxMessage.data[4];

    encoderOffset = ((uint16_t)RxMessage.data[7] << 8) |
                    RxMessage.data[6];

    printf("%d %d %d\n", encoderCurrentPosition, encoderOriginalPosition, encoderOffset);
}

// Set the motor's encoder offset.
void writeEncoderOffset(uint16_t id, uint16_t inputEncoderOffset)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x91;
    TxMessage.data[6] = inputEncoderOffset & 0xFF;
    TxMessage.data[7] = (inputEncoderOffset >> 8) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    encoderOffset = ((uint16_t)RxMessage.data[7] << 8) |
                    RxMessage.data[6];
}

// Write the current position of the motor to the ROM as the motor zero position.
void WritePositionZeroToRom(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x19;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    encoderOffset = ((uint16_t)RxMessage.data[7] << 8) |
                    RxMessage.data[6];

    printf("%d", encoderOffset);
}


// https://stackoverflow.com/questions/14997165/fastest-way-to-get-a-positive-modulo-in-c-c
unsigned modulo( int value, unsigned m) {
    int mod = value % (int)m;
    if (mod < 0) {
        mod += m;
    }
    return mod;
}


// Read the multi-turn angle of the motor.
int32_t readPosition(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x92;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    motorAngle = ((uint64_t)RxMessage.data[7] << 48) |
                 ((uint64_t)RxMessage.data[6] << 40) |
                 ((uint64_t)RxMessage.data[5] << 32) |
                 ((uint64_t)RxMessage.data[4] << 24) |
                 ((uint64_t)RxMessage.data[3] << 16) |
                 ((uint64_t)RxMessage.data[2] << 8) |
                 RxMessage.data[1];

    motorAngle = motorAngle * 0.01;
    motorAngle = modulo(motorAngle*POSITION_FACTOR, 360);
    printf("Multi-turn Angle: %ld\n", motorAngle);
    return motorAngle;
}

// Read the single circle angle of the motor.
uint8_t readCircleAngle(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x94;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    circleAngle = ((uint16_t)RxMessage.data[7] << 8) |
                  RxMessage.data[6];

    circleAngle = circleAngle * 0.01;
    printf("Circle-Angle: %d\n", circleAngle);
    return circleAngle;
}

// Turn off the motor, while clearing the motor operating status and previously received control commands.
void clearState(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x80;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);
}

// Stop the motor, but do not clear the operating state and previously received control commands.
void motorPause(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x81;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);
}

// Resume motor operation from the motor stop command.
void motorResume(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x88;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);
}

// Reads the motor's error status, voltage, temperature and other information.
void readMotorStatus1(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x9A;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    voltage = (((uint16_t)RxMessage.data[4] << 8) |
               RxMessage.data[3]) *
              0.1;
    errorState = RxMessage.data[7];
}

// Clears the error status of the current motor.
void clearErrorFlag(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x9B;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    voltage = (((uint16_t)RxMessage.data[4] << 8) |
               RxMessage.data[3]) *
              0.1;
    errorState = RxMessage.data[7];
}

// Reads the motor temperature, voltage, speed and encoder position.
void readMotorStatus2(uint16_t id)
{
    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0x9C;

    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];

    printf("Velocity: %d\n", velocity);
}

/*
    This writes control commands for the torque and current output.
    iqControlAmp [A]
*/

void writeTorqueCurrent(uint16_t id, int8_t iqControlAmp)
{
    int16_t iqControl = iqControlAmp * 100;

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA1;
    TxMessage.data[4] = iqControl & 0xFF;
    TxMessage.data[5] = (iqControl >> 8) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];
}

/*
    This writes control commands for the speed of the motor output shaft.
    - speedControlRPS [rad/s]
*/

void writeVelocity(uint16_t id, uint8_t speedControlRPS)
{
    int32_t speedControl = speedControlRPS * (180 / M_PI) * 100;

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA2;
    TxMessage.data[4] = speedControl & 0xFF;
    TxMessage.data[5] = (speedControl >> 8) & 0xFF;
    TxMessage.data[6] = (speedControl >> 16) & 0xFF;
    TxMessage.data[7] = (speedControl >> 24) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];
}

/*
    This writes control commands for the position of the motor. The rotation direction is equal to the target position minus the current position.
    - angleControlDegree [degree]
*/

void writePosition1(uint16_t id, int16_t angleControlDegree)
{
    angleControlDegree = angleControlDegree/POSITION_FACTOR;
    int32_t angleControl = (angleControlDegree * 100);

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA3;
    TxMessage.data[4] = angleControl & 0xFF;
    TxMessage.data[5] = (angleControl >> 8) & 0xFF;
    TxMessage.data[6] = (angleControl >> 16) & 0xFF;
    TxMessage.data[7] = (angleControl >> 24) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];
}

/*
    This writes control commands for the position of the motor. The rotation direction is equal to the target position minus the current position. MaxSpeed limits the maximum speed of the motor output shaft rotation.
    - maxSpeedRPS [rad/s]
    - angleControlDegree [degree]
*/

void writePosition2(uint16_t id, uint8_t maxSpeedRPS,
                    int16_t angleControlDegree)
{
    int32_t maxSpeed = maxSpeedRPS * (180 / M_PI);
    printf("Input Angle: %d\n", angleControlDegree);

    angleControlDegree = angleControlDegree/POSITION_FACTOR;
    int32_t angleControl = (angleControlDegree * 100);
    printf("Max Speed: %d\n", maxSpeed);

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

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

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];


}

/*
    This writes control commands for the position of the motor.
        - Spin Direction:
        - 0x00 for clockwise
        - 0x01 for counterclockwise
    - angleControlDegree [degree]
*/

void writePosition3(uint16_t id, uint8_t spinDirection, uint16_t angleControlDegree)
{
    angleControlDegree = angleControlDegree/POSITION_FACTOR;
    int32_t angleControl = (angleControlDegree * 100);

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA5;
    TxMessage.data[1] = spinDirection;
    TxMessage.data[4] = angleControl & 0xFF;
    TxMessage.data[5] = (angleControl >> 8) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((uint16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((uint16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];
}

/*
    This writes control commands for the position of the motor. MaxSpeed limits the maximum speed of the motor output shaft rotation.
    - Spin Direction:
        - 0x00 for clockwise
        - 0x01 for counterclockwise
    - maxSpeedRPS [rad/s]
    - angleControlDegree [degree]
*/

void writePosition4(uint16_t id, uint8_t spinDirection,
                    uint8_t maxSpeedRPS, uint16_t angleControlDegree)
{
    int32_t maxSpeed = maxSpeedRPS * (180 / M_PI);

    angleControlDegree = angleControlDegree/POSITION_FACTOR;
    int32_t angleControl = (angleControlDegree * 100);

    CAN_Message TxMessage = {0};
    CAN_Message RxMessage = {0};

    setCommonFields(&TxMessage, id);
    TxMessage.data[0] = 0xA6;
    TxMessage.data[1] = spinDirection;
    TxMessage.data[2] = maxSpeed & 0xFF;
    TxMessage.data[3] = (maxSpeed >> 8) & 0xFF;
    TxMessage.data[4] = angleControl & 0xFF;
    TxMessage.data[5] = (angleControl >> 8) & 0xFF;
    WriteCmd(&TxMessage);

    sleep(1);
    ReadCmd(&RxMessage);

    temperature = RxMessage.data[1] * (9 / 5) + 32;
    torqueCurrent = ((int16_t)RxMessage.data[3] << 8) |
                    RxMessage.data[2];
    velocity = ((int16_t)RxMessage.data[5] << 8) |
               RxMessage.data[4];
    encoderCurrentPosition = ((uint16_t)RxMessage.data[7] << 8) |
                             RxMessage.data[6];
}
