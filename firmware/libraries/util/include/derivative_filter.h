#ifndef DERIVATIVE_FILTER_H_INCLUDED
#define DERIVATIVE_FILTER_H_INCLUDED

// Implement a discrete time filter for speed/derivative of the following form
// V(k) = a0*P(k) + a1*P(k-1) + a2*P(k-2) - b1*V(k-1) - b2*V(k-2).
// It approximates a continuous time 2nd order low-pass with a derivative of the form
// V(s)/P(s) = s*wn^2/(s^2 + 2*zeta*wn*s +wn^2)
class DerivativeFilter
{
  public: // methods

    // Constructor
    DerivativeFilter(float sample_time, float cuttoff_frequency, float damping_ratio);

    // Return time derivative estimate of input. Needs to be called close to the sample
    // time specified in the constructor for the filter to work correctly.
    float calculate(float position);

    // Reset previous values of filters to 0.
    void reset(void);

  private: // fields

    // Filter coefficients
    float a0_, a1_, a2_, b1_, b2_;

    // Previous values. V for velocity (output) and P for position (input).
    float vk1_, vk2_, pk1_, pk2_;
};

#endif
