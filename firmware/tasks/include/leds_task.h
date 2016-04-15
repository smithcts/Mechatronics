#ifndef LEDS_TASK_H_INCLUDED
#define LEDS_TASK_H_INCLUDED

// Includes
#include "digital_out.h"
#include "green_leds.h"
#include "periodic_task.h"
#include "user_leds.h"

// Update LEDs on robot to match robot mode and battery voltage.
class LedsTask : public Scheduler::PeriodicTask
{
public: // methods

    // Constructor
    LedsTask(float frequency);

    // Set new green LED pattern next time task runs.
    void requestNewLedGreenPattern(uint8_t count);

private: // methods

    // Toggle LEDs to indicate when the robot is in different modes and also
    // if the battery is getting low.
    virtual void run(void);

    // Not currently implemented.
    virtual void initialize(void) {}

    // Ran at a slower rate than main task.  In charge of updating LEDs that shouldn't change really fast.
    void slowRun(void);

    // Read in data from global objects (globs).
    void readNewData(void);

private: // fields

    // Set to true once another task requested a new green LED pattern.  Will stay true until reset.
    bool new_green_pattern_requested_;

    // Green LED count pattern requested by another task.
    uint8_t requested_green_pattern_;

    // If set to true then the LEDs should show the battery state, otherwise will turn off all LEDs
    // to show that the program is still running.
    bool flash_state_is_off;

    // Interfaces to control LEDs on robot.
    UserLeds user_leds_;
    GreenLeds green_leds_;

    // Globs from other tasks.
    glo_modes_t modes_;
    glo_analog_t analog_;

};

// Task instance - defined in main.cpp
extern LedsTask leds_task;

#endif
