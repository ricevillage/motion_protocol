/*
# Copyright 2021 Xilinx Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

/*
* blink.h - the header file with the ioctl definitions.
*
* The declarations here have to be in a header file, because
* they need to be known both to the kernel module
* (in chardev.c) and the process calling ioctl (ioctl.c)
*/
#ifndef SBUS_H
#define SBUS_H
#include <linux/ioctl.h>
/*
* The major device number. We can't rely on dynamic
* registration any more, because ioctls need to know
* it.
*/
#define MAGIC_NUM 237
/*
* TURN ON LED ioctl
*/
#define IOCTL_ON_LED _IOR(MAGIC_NUM, 0, char *)
/*
* _IOR means that we're creating an ioctl command
* number for passing information from a user process
* to the kernel module.
*
* The first arguments, MAGIC_NUM, is the major device
* number we're using.
*
* The second argument is the number of the command
* (there could be several with different meanings).
*
* The third argument is the type we want to get from
* the process to the kernel.
*/
/*
* STOP LED BLINK ioctl
*/
#define IOCTL_STOP_LED _IOR(MAGIC_NUM, 1, char *)

#define IOCTL_CH_1_2 _IOR(MAGIC_NUM, 0, unsigned int *)
#define IOCTL_CH_3_4 _IOR(MAGIC_NUM, 1, unsigned int *)
#define IOCTL_CH_5_6 _IOR(MAGIC_NUM, 2, unsigned int *)
#define IOCTL_CH_7_8 _IOR(MAGIC_NUM, 3, unsigned int *)
#define IOCTL_CH_9_10 _IOR(MAGIC_NUM, 4, unsigned int *)
#define IOCTL_CH_11_12 _IOR(MAGIC_NUM, 5, unsigned int *)
#define IOCTL_CH_13_14 _IOR(MAGIC_NUM, 6, unsigned int *)
#define IOCTL_CH_15_16 _IOR(MAGIC_NUM, 7, unsigned int *)
#define IOCTL_CH_ERROR _IOR(MAGIC_NUM, 8, unsigned int *)

#define DEBUG

#define DEVICE_FILE_NAME "/dev/SBUS_Dev"
#endif
