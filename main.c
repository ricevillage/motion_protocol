#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "CAN.h"

#define MOTOR_ID1 0x141
#define MOTOR_ID2 0x142

int main()
{
    Initialize();
    clearState(MOTOR_ID1);
    clearState(MOTOR_ID2);

    while(1) {
		writePosition(MOTOR_ID1, 500, 0);
		writePosition(MOTOR_ID2, 500, 0);
		writePosition(MOTOR_ID1, 500, 36000);
		writePosition(MOTOR_ID2, 500, 36000);
    }

    clearState(MOTOR_ID1);
    clearState(MOTOR_ID2);
    Cleanup();
    return 0;
}
