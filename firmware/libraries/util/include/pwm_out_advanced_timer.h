#ifndef PWM_OUT_ADVANCED_TIMER_H_INCLUDED
#define PWM_OUT_ADVANCED_TIMER_H_INCLUDED

// Includes
#include <cstdint>

// Implement simple PWM output on an advanced timer (TIM1).
class PwmOutAdvancedTimer
{
  public: // methods

    // Constructor
    PwmOutAdvancedTimer(void);

    // Update the channel (1, 2, 3, or 4) to have the specified duty cycle
    // which is capped to be between 0 and 1.
    void setDuty(float duty, uint8_t channel_num);

  private: // fields

    // ARR clock counts to get period
    uint32_t auto_reload_reg_;
};

#endif
