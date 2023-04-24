/*
 * Project github: https://github.com/ricevillage/motion_protocol
 */

#include "PmodCAN.h"
#include "unistd.h"
#include "stdio.h"
#include "MotionProtocol.h"
#include "MotionController.h"
#include "CAN.h"

/* sbus includes */
#include "sbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> /* open */
#include <unistd.h> /* exit */
#include <sys/ioctl.h> /* ioctl */


extern int currentState;

void updateChannelState(int file_desc)
{
	int channels;
	int ret_val;
//	ret_val = ioctl(file_desc, IOCTL_CH_1_2,&channels);
//	printf("Channels1&2:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_3_4,&channels);
//	printf("Channels3&4:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_5_6,&channels);
//	printf("Channels5&6:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_7_8,&channels);
//	printf("Channels7&8:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_9_10,&channels);
//	printf("Channels9&10:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_11_12,&channels);
//	printf("Channels11&12:%X\n",channels);
//	ret_val = ioctl(file_desc, IOCTL_CH_13_14,&channels);
//	printf("Channels13&14:%X\n",channels);
	ret_val = ioctl(file_desc, IOCTL_CH_15_16,&channels);
	printf("Channels15&16:%X\n",channels);

	currentState = channels;
}

int main()
{
    Initialize(CAN_BUS1);
    Initialize(CAN_BUS2);

	int exitflag=1;
	int file_desc;
	int channels;
	int ret_val;

	printf("################################ \n\r");
	printf("      SBUS App  \n\r");
	printf("GO COOGS!\n");
	printf("################################ \n\r");
	file_desc = open(DEVICE_FILE_NAME, O_RDWR | O_SYNC);
	if (file_desc < 0)
	{
		printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
		exit(-1);
	}
	while (exitflag)
	{

		updateChannelState(file_desc);

		switch (currentState)
		{
			case INITIAL_STATE:
				break;
			case STANDUP_STATE:
				printf("stand up\n");
				robotStandUpCommand();
				break;
			case SITDOWN_STATE:
				printf("sit down\n");
				robotSitDownCommand();
				break;
			case FORWARD_GAIT_STATE:
				printf("gait forward\n");
				legGaitForward(file_desc);
				break;
//			case BACKWARD_GAIT_STATE:
//				printf("gait backwards\n");
//				legGaitBackward(file_desc);
//				break;
			case STOP_STATE:
				printf("STOP!\n");
				StopAllMotors();
				break;
			default:
				break;
		}

		ret_val = ioctl(file_desc, IOCTL_CH_ERROR,&channels);
		printf("ERROR:%X\n",channels);
		sleep(1);
	}

	Cleanup(CAN_BUS1);
	Cleanup(CAN_BUS2);

	close(file_desc);
    return ret_val;
}
