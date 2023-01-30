#include <stdio.h>
#include "MotionProtocol.h"

#define PID_ID 0x141

int main()
{
    Initialize();
    readPID(PID_ID);
    Cleanup();
    return 0;
}