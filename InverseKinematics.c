#include "InverseKinematics.h"

void calculateLegJointAngles(uint16_t id, double z, JointAngles *angles)
{
    // // calculate leg length based on shin/thigh length and knee and hip angle
    // double b_squared = UPPER_LINK_LENGTH * UPPER_LINK_LENGTH;
    // double c_squared = z * z;
    // double a_squared = LOWER_LINK_LENGTH * LOWER_LINK_LENGTH;
    // double numerator = b_squared + c_squared - a_squared;
    // double denominator = 2 * UPPER_LINK_LENGTH * z;
    // double cosineOfHipPitchAngle = numerator / denominator;

    // // check for valid input to acos function
    // if (cosineOfHipPitchAngle < -1.0 || cosineOfHipPitchAngle > 1.0)
    // {
    //     // handle invalid input as desired (e.g. set default value)
    //     angles->knee_pitch = 0.0;
    //     return;
    // }

    // double hipPitchAngle = acos(cosineOfHipPitchAngle); // radians
    double hipPitchAngle = readPosition(id);
    double kneePitchAngle = M_PI - 2 * hipPitchAngle; // radians

    // convert radians to degrees
    double hipPitchAngleDegrees = hipPitchAngle * (180.0 / M_PI); // degrees
    double kneeAngleDegrees = kneePitchAngle * (180.0 / M_PI);    // degrees

    angles->hip_pitch = hipPitchAngleDegrees;
    angles->knee_pitch = kneeAngleDegrees;

    printf("Angle from motor: %f", hipPitchAngleDegrees);
    printf("%f", kneeAngleDegrees);
}