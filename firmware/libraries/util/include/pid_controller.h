#ifndef PID_CONTROLLER_H_INCLUDED
#define PID_CONTROLLER_H_INCLUDED

// Includes
#include "globs.h"

// Proportional Integral Derivative Controller
class PidController
{
  public:

    // Constructor
    PidController(float kp, float ki, float kd,
                  float integral_lolimit, float integral_hilimit,
                  float output_lolimit, float output_hilimit);

    // Return output of controller for updated 'error' (difference between desired and actual)
    // derivative of error and dt (time since last calculate() call)
    float calculate(float error, float derivative, float dt);

    // Same as above method but will automatically calculate the derivative, which will usually be
    // noisy so it's best to pass in your own derivative if you can.
    float calculate(float error, float dt);

    // Reset integral term to zero to avoid windup.
    void resetIntegral(void) { integral_ = 0.0f; };

    // To keep in sync with global parameters (for sending/receiving from GUI)
    // Setting the parameters should only be done through the receive task
    // so that the global object will be updated too.
    void set(glo_pid_params_t const & params);
    void get(glo_pid_params_t & params) const;

  private:

    // Parameters
    float kp_, ki_, kd_;                         // controller gains
    float integral_hilimit_, integral_lolimit_;  // integrator antiwindup limits
    float hilimit_, lolimit_;                    // output limits

    // Integral portion of the controller
    float integral_;

    // Last error passed to 'calculate' method.  Used for calculating a basic derivative (i.e. dx/dt)
    float last_error_;
};

#endif
