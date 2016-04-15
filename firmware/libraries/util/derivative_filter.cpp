//Includes
#include "derivative_filter.h"

//*****************************************************************************
DerivativeFilter::DerivativeFilter(float sample_time, float cuttoff_frequency, float damping_ratio)
{
    float wn = 2 * 3.14159 * cuttoff_frequency;
    float T = sample_time;
    float zeta = damping_ratio;

    // Set filter coefficients
    float b0 = (T*T*wn*wn + 4*zeta*T*wn + 4);
    b1_ = (2*T*T*wn*wn - 8)/b0;
    b2_ = (T*T*wn*wn - 4*zeta*T*wn + 4)/b0;
    a0_ = 2*T*wn*wn/b0;
    a1_ = 0/b0;
    a2_ = (-2*T*wn*wn)/b0;

    // Initialize previous values
    reset();
}

//*****************************************************************************
void DerivativeFilter::reset(void)
{
    vk1_ = vk2_ = pk1_ = pk2_ = 0;
}

//*****************************************************************************
float DerivativeFilter::calculate(float position)
{
    float velocity = a0_*position + a1_*pk1_ + a2_*pk2_ - b1_*vk1_ - b2_*vk2_;

    pk2_ = pk1_;
    pk1_ = position;
    vk2_ = vk1_;
    vk1_ = velocity;

    return velocity;
}
