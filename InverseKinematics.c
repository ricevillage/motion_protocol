#include "InverseKinematics.h"

void calculateLegJointAngles(uint16_t id1, uint16_t id2, double z)
{
    // calculate leg length based on shin/thigh length and knee and hip angle
    double b_squared = UPPER_LINK_LENGTH * UPPER_LINK_LENGTH;
    double c_squared = z * z;
    double a_squared = LOWER_LINK_LENGTH * LOWER_LINK_LENGTH;
    double numerator = b_squared + c_squared - a_squared;
    double denominator = 2 * UPPER_LINK_LENGTH * z;
    double cosineOfHipPitchAngle = numerator / denominator;

    // check for valid input to acos function
    if (cosineOfHipPitchAngle < -1.0 || cosineOfHipPitchAngle > 1.0)
        return;

    double hipPitchAngle = acos(cosineOfHipPitchAngle); // radians
    double kneePitchAngle = M_PI - 2 * hipPitchAngle;   // radians

    // convert radians to degrees
    double hipPitchAngleDegrees = hipPitchAngle * (180.0 / M_PI); // degrees
    double kneeAngleDegrees = kneePitchAngle * (180.0 / M_PI);    // degrees

    kneeAngleDegrees = kneeAngleDegrees;
    hipPitchAngleDegrees = hipPitchAngleDegrees;

    //    printf("Hip Pitch Angle: %f\n", hipPitchAngleDegrees);
    //    printf("Knee Angle: %f\n", kneeAngleDegrees);

    int32_t currentHipPitchAngle = readPosition(id1);
    int32_t currentKneeAngle = readPosition(id2);

    int32_t newHipPitchAngle = clampAngle(hipPitchAngleDegrees - currentHipPitchAngle);
    int32_t newKneeAngle = clampAngle(kneeAngleDegrees - currentKneeAngle);

    printf("Hip Pitch Angle: %d\n", newHipPitchAngle);
    printf("Knee Angle: %d\n", newKneeAngle);

    writePosition2(id1, 5, newHipPitchAngle);
    writePosition2(id2, 5, newKneeAngle);

    sleep(1);
}

int32_t clampAngle(int32_t angle)
{
    return (angle % 360) + (angle < 0 ? 360 : 0);
}

// https://stackoverflow.com/a/3380723
double acos(double x)
{
    return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
}
