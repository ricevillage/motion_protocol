#include <stdio.h>
#include "MotionProtocol.h"

#define MOTOR_ID 0x141

int main()
{
    Initialize();
    readPID(MOTOR_ID);
    Cleanup();
    return 0;
}