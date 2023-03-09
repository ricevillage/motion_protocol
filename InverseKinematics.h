
#ifndef Inverse_Kinematics_H
#define Inverse_Kinematics_H

#include "cmath"
#include <stdint.h>
#include "MotionProtocol.h"

#define _USE_MATH_DEFINES

#define M_PI 3.14159265358979323846

// #define UPPER_LINK_LENGTH 0.22
// #define LOWER_LINK_LENGTH 0.22

// Struct for storing joint angles
typedef struct
{
    double hip_roll;
    double hip_pitch;
    double knee_pitch;
} JointAngles;

void calculateLegJointAngles(double z, JointAngles *angles);

#endif // Inverse_Kinematics_H
