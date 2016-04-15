#ifndef COMPLEMENTARY_FILTER_H_INCLUDED
#define COMPLEMENTARY_FILTER_H_INCLUDED

// Includes
#include <cstdint>

// Combine gyro and accelerometer measurements by using a high pass filter on the rate gyros
// and a low pass filter on the orientation estimated from the accels.  This is implemented
// using 3 Proportional-Integral (PI) controllers for the accels and using the gyros as a
// feed-forward term.  The state is represented as a quaternion since this representation
// doesn't suffer from singularities like euler angles and axis-angle do.
class ComplementaryFilter
{
public: // methods

    // Constructor
    explicit ComplementaryFilter(void);

    // Run the filter using new gyro and accel data.  'dt' is the time since last call in seconds.
    // Call getAttitude() to get the updated quaternion.
    void update(float dt, const float gyros[3], const float accels[3]);

    // Set a new state for the filter.
    void setAttitude(const float quaternion[4]);

    // Get most recent state of filter.
    void getAttitude(float quaternion[4]);

private: // fields

    // Proportional and integral attitude filter gains.
    float kpa_, kia_;

    // Attitude state variables represented as a quaternion.
    float quat_[4];

    // Integral state for 3 PI controllers (for 3 axis)
    float att_integral_[3];

};

#endif
