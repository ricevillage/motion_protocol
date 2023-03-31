#include "InverseKinematics.h"

/*
    This function takes in two motor IDs, knee and thigh, and a desired knee joint height. It calculates the required hip and knee pitch angles using the law of cosines, reads the current motor positions, and calculates the new angles required to move the knee joint to the desired height.

    additionalHipPitchAngle
    - default value is 0
    - necessary for moveLegInXDirection()
*/

void moveLegInZDirection(uint16_t id1, uint16_t id2, double z, double additionalHipPitchAngle)
{
    // Calculate the length of the leg based on the shin/thigh length and knee and hip angle
    // Use the law of cosines to calculate the angle between the upper link and the height of the foot from the base
    double b_squared = UPPER_LINK_LENGTH * UPPER_LINK_LENGTH;
    double c_squared = z * z;
    double a_squared = LOWER_LINK_LENGTH * LOWER_LINK_LENGTH;
    double numerator = b_squared + c_squared - a_squared;
    double denominator = 2 * UPPER_LINK_LENGTH * z;
    double cosineOfHipPitchAngle = numerator / denominator;

    // Check if the angle is within the valid range for the inverse cosine function
    // If not, return without performing any further calculations
    if (cosineOfHipPitchAngle < -1.0 || cosineOfHipPitchAngle > 1.0)
    {
        return;
    }

    // Calculate the hip pitch angle and knee pitch angle in radians
    double hipPitchAngle = acos(cosineOfHipPitchAngle);
    double kneePitchAngle = M_PI - 2 * hipPitchAngle;

    // Convert radians to degrees
    double hipPitchAngleDegrees = hipPitchAngle * (180.0 / M_PI);
    double KneePitchAngleDegrees = kneePitchAngle * (180.0 / M_PI);

    // Read the current position of the motors
    double currentHipPitchAngleDegrees = readPosition(id1);
    double currentKneePitchAngleDegrees = readPosition(id2);

    // Calculate the new angles required to reach the desired knee joint height
    // Subtract 45 degrees from the hip pitch angle and 90 degrees from the knee pitch angle to adjust for initial positions
    double newHipPitchAngleDegrees = currentHipPitchAngleDegrees + hipPitchAngleDegrees + additionalHipPitchAngle - 45;
    double newKneePitchAngleDegrees = currentKneePitchAngleDegrees + KneePitchAngleDegrees - 90;

    // Print the new angles for debugging purposes
    printf("Hip Pitch Angle: %f\n", newHipPitchAngleDegrees);
    printf("Knee Angle: %f\n", newKneePitchAngleDegrees);

    // Write the new angles to the motors
    writePosition2(id1, 100, newHipPitchAngleDegrees);
    writePosition2(id2, 100, newKneePitchAngleDegrees);
}

/*
    This function takes in two motor IDs, id1 and id2, a desired leg width (x), and initial leg height (z). It calculates a new leg height based on the desired leg width and an additional angle to add to the hip joint. Finally, it calls the moveLegInZDirection function using the new variables.
*/

void moveLegInXDirection(double x, double z)
{
    // Calculate hip pitch angle in degrees from leg width and initial leg height
    double hipPitchAngle2 = atan(x / z);
    double hipPitchAngleDegrees2 = hipPitchAngle2 * (180.0 / M_PI);

    // Calculate new leg height from leg width and hip pitch angle
    double newZMagnitude = z / cos(hipPitchAngle2);

    printf("hip2: %f\n", hipPitchAngleDegrees2);
    printf("new height: %f\n", newZMagnitude);

    // Call moveLegInZDirection with the new leg height and hip pitch angle
    moveLegInZDirection(newZMagnitude, hipPitchAngleDegrees2);
}

// Approximation of the inverse cosine function using a polynomial
// https://stackoverflow.com/a/3380723
double acos(double x)
{
    return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
}
