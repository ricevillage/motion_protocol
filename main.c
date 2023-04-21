#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "MotionController.h"
#include "CAN.h"

/*
 * Project github: https://github.com/ricevillage/motion_protocol
 */

int main()
{
  printf("Go coogs!\n");
  Initialize(CAN_BUS1);
  Initialize(CAN_BUS2);

//  performRobotStandTest();
  legGait1();

  Cleanup(CAN_BUS1);
  Cleanup(CAN_BUS2);

  return 0;
}
