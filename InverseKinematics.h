
#ifndef Inverse_Kinematics_H
#define Inverse_Kinematics_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//#include <math.h>
#include "MotionProtocol.h"

#define M_PI 3.14159265358979323846

#define UPPER_LINK_LENGTH 0.22
#define LOWER_LINK_LENGTH 0.22

void moveKneeToDesiredHeight(uint16_t id1, uint16_t id2, double z);
double acos(double x);

#endif // Inverse_Kinematics_H
