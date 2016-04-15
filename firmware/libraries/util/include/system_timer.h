#ifndef SYSTEM_TIMER_H_INCLUDED
#define SYSTEM_TIMER_H_INCLUDED

// Includes
#include <cstdint>

// Systick timer with sub-microsecond resolution.
class SystemTimer
{
  public: // methods

    // Constructor.  Setup up systick timer.
    SystemTimer(void);

    // Return number of clock ticks since timer was created.
    uint64_t ticks(void);

    // Return time in seconds since timer was created.
    double seconds(void) { return ticks() * seconds_per_tick_; }

    // Return rate that timer is running at.
    uint32_t frequency(void) const { return timer_frequency_; }

    // Return after the specified number of seconds has elapsed.
    void busyWait(double seconds_to_wait);

  public: // fields

    // How many times the timer has reached 0 and reset back.
    // Should only be updated from systick update interrupt.
    // 64 bit to make calculations of current ticks correct.
    uint64_t rollover_count_;

  private: // fields

    // Number of clock ticks that timer will reset to after reaching 0.
    uint32_t reload_value_;

    // Rate that the timer is running at.
    uint32_t timer_frequency_;

    // How many seconds between ticks.
    double seconds_per_tick_;

    // Last value calculated from ticks() method.  Used to detect timer overflow before interrupt has fired.
    uint64_t last_reported_ticks_;

};

extern SystemTimer sys_timer;

#endif
