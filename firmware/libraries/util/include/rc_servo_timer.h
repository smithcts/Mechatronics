#ifndef RC_SERVO_TIMER_H_INCLUDED
#define RC_SERVO_TIMER_H_INCLUDED

// Includes
#include <cstdint>

// Implement 50 Hz, 1-2 ms pulse width, timer on TIM9 Ch2
// with pin PE6 (could use PE5 on Ch1)
class RcServoTimer
{
  public: // methods

    // Constructor
    RcServoTimer(void);

    // Update the pulse width.  Capped to be between 0.75ms and 2.25ms.
    void setPulse(float pulse_width_ms);

  private: // fields

    // ARR clock counts to get period
    uint16_t frequency_;
    uint32_t auto_reload_reg_;
};

#endif
