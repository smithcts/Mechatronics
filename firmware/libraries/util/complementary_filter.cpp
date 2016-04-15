// Includes
#include <math.h>
#include <string.h>

#include "complementary_filter.h"
#include "coordinate_conversions.h"
#include "physical_constants.h"

//*****************************************************************************
ComplementaryFilter::ComplementaryFilter(void)
{
    float cuttoff = 6.28f / 6.0f;
    kpa_ = cuttoff;
    kia_ = cuttoff * cuttoff;

    quat_[0] = 1.0f;
    quat_[1] = quat_[2] = quat_[3] = 0.0f;

    memset(att_integral_, 0, sizeof(att_integral_));
}

//*****************************************************************************
void ComplementaryFilter::update(float dt, const float gyros[3], const float accels[3])
{
    float error[3];
    float accels_mag;
    float predicted_accels_mag;
    float accels_direction[3];
    float accels_predicted[3];
    float rbe[3][3]; // Rotation matrix from body frame to earth frame.
    float up[3];

    // Find the current rotation matrix using the state quaternion.
    // Up is the positive ENU z-axis in the body frame this is positive
    // of the last column of rbe.
    quaternion_2_R(quat_, rbe);
    up[0] = rbe[0][2];
    up[1] = rbe[1][2];
    up[2] = rbe[2][2];

    //***************************************
    // Run the complementary attitude filter
    //***************************************

    // Predicted accels = corrections + gravity, corrections should be zero if not used
    accels_predicted[0] = up[0] * GRAVITY;
    accels_predicted[1] = up[1] * GRAVITY;
    accels_predicted[2] = up[2] * GRAVITY;

    // Find magnitudes and unit vectors, if either is small avoid division by zero
    predicted_accels_mag = vector_magnitude(accels_predicted);
    accels_mag = vector_magnitude(accels);

    if ( (fabsf(predicted_accels_mag) < 1e-30) || (fabsf(accels_mag) < 1e-30) )
    {
        // Skip the correction this time by zeroing one of the vectors
        accels_direction[0] = accels_direction[1] = accels_direction[2] = 0.0f;
    }
    else
    {
        // Calculate direction of accels and predicted accels
        accels_direction[0] = accels[0] / accels_mag;
        accels_direction[1] = accels[1] / accels_mag;
        accels_direction[2] = accels[2] / accels_mag;
        accels_predicted[0] /= predicted_accels_mag;
        accels_predicted[1] /= predicted_accels_mag;
        accels_predicted[2] /= predicted_accels_mag;
    }

    // Find rotation (error term for PI) in direction from predicted towards measured
    cross_product(accels_direction, accels_predicted, error);

    // Update integral states for PI controllers
    // TODO - do we need to limit windup?
    att_integral_[0] += error[0] * dt;
    att_integral_[1] += error[1] * dt;
    att_integral_[2] += error[2] * dt;

    // Add PI corrections to gyros
    float wx = gyros[0] + kpa_*error[0] + kia_*att_integral_[0];
    float wy = gyros[1] + kpa_*error[1] + kia_*att_integral_[1];
    float wz = gyros[2] + kpa_*error[2] + kia_*att_integral_[2];

    // Do numerical integration, angular rates = gyros + corrections
    float qdot[4];
    qdot[0] = (-quat_[1]*wx - quat_[2]*wy - quat_[3]*wz) / 2.0f;
    qdot[1] = ( quat_[0]*wx - quat_[3]*wy + quat_[2]*wz) / 2.0f;
    qdot[2] = ( quat_[3]*wx + quat_[0]*wy - quat_[1]*wz) / 2.0f;
    qdot[3] = (-quat_[2]*wx + quat_[1]*wy + quat_[0]*wz) / 2.0f;

    // Take a time step
    quat_[0] += qdot[0] * dt;
    quat_[1] += qdot[1] * dt;
    quat_[2] += qdot[2] * dt;
    quat_[3] += qdot[3] * dt;

    // Perform cleanup on quaternion.
    setAttitude(quat_);
}

//*****************************************************************************
void ComplementaryFilter::setAttitude(const float quaternion[4])
{
    // Update local field after cleaning up the passed in quaternion by making sure:
    //  1) It's first element is positive.  If it's not then we have to flip all the signs.
    //  2) It's normalized.
    float quat_sign = (quaternion[0] < 0) ? -1 : 1;
    float quat_mag = quat_magnitude(quaternion);
    for (uint8_t i = 0; i < 4; i++)
    {
        quat_[i] = (quaternion[i] * quat_sign) / quat_mag;
    }
}

//*****************************************************************************
void ComplementaryFilter::getAttitude(float quaternion[4])
{
   quaternion[0] = quat_[0];
   quaternion[1] = quat_[1];
   quaternion[2] = quat_[2];
   quaternion[3] = quat_[3];
}
