#include "InverseKinematics.h"

/*
    This function takes in two motor IDs, knee and thigh, and a desired knee joint height. It calculates the required hip and knee pitch angles using the law of cosines, reads the current motor positions, and calculates the new angles required to move the knee joint to the desired height.
*/

void moveKneeToDesiredHeight(uint16_t id1, uint16_t id2, double z)
{
    // calculate leg length based on shin/thigh length and knee and hip angle
    // Using the law of cosines to calculate the angle between the upper link
    // and the height of the foot from the base
    double b_squared = UPPER_LINK_LENGTH * UPPER_LINK_LENGTH;
    double c_squared = z * z;
    double a_squared = LOWER_LINK_LENGTH * LOWER_LINK_LENGTH;
    double numerator = b_squared + c_squared - a_squared;
    double denominator = 2 * UPPER_LINK_LENGTH * z;
    double cosineOfHipPitchAngle = numerator / denominator;

    // check if the angle is within the valid range for the inverse cosine function
    if (cosineOfHipPitchAngle < -1.0 || cosineOfHipPitchAngle > 1.0)
    {
        return;
    }

    double hipPitchAngle = acos(cosineOfHipPitchAngle); // radians
    double kneePitchAngle = M_PI - 2 * hipPitchAngle;   // radians

    // convert radians to degrees
    double hipPitchAngleDegrees = hipPitchAngle * (180.0 / M_PI); // degrees
    double kneeAngleDegrees = kneePitchAngle * (180.0 / M_PI);    // degrees

    // reading the current position of the motors
    int32_t currentHipPitchAngle = readPosition(id1);
    int32_t currentKneeAngle = readPosition(id2);

    // calculating the new angles required to reach the desired knee joint height
    int32_t newHipPitchAngle = clampAngle(hipPitchAngleDegrees - currentHipPitchAngle);
    int32_t newKneeAngle = clampAngle(kneeAngleDegrees - currentKneeAngle);

    // printing the new angles for debugging purposes
    printf("Hip Pitch Angle: %d\n", newHipPitchAngle);
    printf("Knee Angle: %d\n", newKneeAngle);

    // writing the new angles to the motors
    writePosition2(id1, 5, newHipPitchAngle);
    writePosition2(id2, 5, newKneeAngle);
}

// Function to clamp angle values between 0 and 359 degrees
int32_t clampAngle(int32_t angle)
{
    return (angle % 360) + (angle < 0 ? 360 : 0);
}

// Approximation of the inverse cosine function using a polynomial
// https://stackoverflow.com/a/3380723
double acos(double x)
{
    return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
}
