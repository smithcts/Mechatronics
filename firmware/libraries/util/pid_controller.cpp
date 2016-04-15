// Includes
#include "pid_controller.h"

//******************************************************************************
PidController::PidController(float kp, float ki, float kd,
                             float integral_lolimit, float integral_hilimit,
                             float output_lolimit, float output_hilimit)
{
    // Initialize parameters and integral state.
    kp_               = kp;
    ki_               = ki;
    kd_               = kd;
    integral_lolimit_ = integral_lolimit;
    integral_hilimit_ = integral_hilimit;
    lolimit_          = output_lolimit;
    hilimit_          = output_hilimit;
    integral_         = 0.0f;
    last_error_       = 0.0f;
}

//******************************************************************************
float PidController::calculate(float error, float derivative, float dt)
{
    // Update the integral state.  Multiply by the integral gain (ki) so that when we saturate the
    // integral term below we can control the maximum effect the integral has on the total output.
    integral_ += ki_ * error * dt;

    // Saturate integral term to avoid windup.
    if (integral_ < integral_lolimit_) { integral_ = integral_lolimit_; }
    if (integral_ > integral_hilimit_) { integral_ = integral_hilimit_; }

    float output = kp_*error + integral_ + kd_*derivative;

    // Saturate output at limits.
    if (output < lolimit_) { output = lolimit_; }
    if (output > hilimit_) { output = hilimit_; }

    // Save error in case we need it to calculate derivative.
    last_error_ = error;

    return output;
}

//******************************************************************************
float PidController::calculate(float error, float dt)
{
    if (dt == 0) { return 0; } // protect against division by zero

    float derivative = (error - last_error_) / dt;

    return calculate(error, derivative, dt);
}

//******************************************************************************
void PidController::set(glo_pid_params_t const & params)
{
    kp_ = params.kp;
    kd_ = params.kd;
    ki_ = params.ki;
    lolimit_ = params.lolimit;
    hilimit_ = params.hilimit;
    integral_lolimit_ = params.integral_lolimit;
    integral_hilimit_ = params.integral_hilimit;
}

//******************************************************************************
void PidController::get(glo_pid_params_t & params) const
{
    params.kp = kp_;
    params.kd = kd_;
    params.ki = ki_;
    params.lolimit = lolimit_;
    params.hilimit = hilimit_;
    params.integral_lolimit = integral_lolimit_;
    params.integral_hilimit = integral_hilimit_;
}
