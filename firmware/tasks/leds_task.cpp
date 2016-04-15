// Includes
#include "globs.h"
#include "leds_task.h"
#include "robot_settings.h"
#include "status_update_task.h"
#include "util_assert.h"

//******************************************************************************
LedsTask::LedsTask(float frequency) :
        PeriodicTask("Leds", TASK_ID_LED, frequency),
        new_green_pattern_requested_(false),
        requested_green_pattern_(0),
        flash_state_is_off(false)
{
}

//******************************************************************************
void LedsTask::readNewData(void)
{
    glo_modes.read(&modes_);
    glo_analog.read(&analog_);
}

//******************************************************************************
void LedsTask::run(void)
{
    readNewData();

    if (modes_.main_mode != MAIN_MODE_LINE_FOLLOWING)
    {
        // Set the green LEDs at the top of the board to show what mode the robot is in.
        uint8_t green_led_pattern = 1 << modes_.main_mode;
        green_leds_.set(green_led_pattern);
    }
    else
    {
        // In line following mode so reflect state of IR sensors.
        green_leds_.set(requested_green_pattern_);
    }

    if (throttleHz(2))
    {
        slowRun();
    }
}

//******************************************************************************
void LedsTask::slowRun(void)
{
    // Set to true if battery voltage is too low to operate robot.
    bool cricitial_battery = false;

    // Toggle flash state for flashing LEDs to show that program is running.
    flash_state_is_off = !flash_state_is_off;

    if (flash_state_is_off)
    {
        user_leds_.clear(USER_LED_GREEN);
        user_leds_.clear(USER_LED_YELLOW);
        user_leds_.clear(USER_LED_ORANGE);
        user_leds_.clear(USER_LED_RED);
    }
    else
    {
        if (analog_.battery_voltage > FULL_BATTERY_VOLTAGE)
        {
            user_leds_.set(USER_LED_GREEN);
            user_leds_.clear(USER_LED_YELLOW);
            user_leds_.clear(USER_LED_ORANGE);
            user_leds_.clear(USER_LED_RED);
        }
        else if (analog_.battery_voltage > LOW_BATTERY_VOLTAGE)
        {
            user_leds_.clear(USER_LED_GREEN);
            user_leds_.set(USER_LED_YELLOW);
            user_leds_.clear(USER_LED_ORANGE);
            user_leds_.clear(USER_LED_RED);
        }
        else if (analog_.battery_voltage > CRITICAL_BATTERY_VOLTAGE)
        {
            user_leds_.clear(USER_LED_GREEN);
            user_leds_.clear(USER_LED_YELLOW);
            user_leds_.set(USER_LED_ORANGE);
            user_leds_.clear(USER_LED_RED);
        }
        else
        {
            cricitial_battery = true;
            user_leds_.clear(USER_LED_GREEN);
            user_leds_.clear(USER_LED_YELLOW);
            user_leds_.clear(USER_LED_ORANGE);
            user_leds_.set(USER_LED_RED);
        }
    }

    if (cricitial_battery)
    {
        status_update_task.setErrorCodes(ERROR_CODE_CRICITAL_BATTERY);
    }
    else // battery is ok so make sure error code isn't set.
    {
        status_update_task.clearErrorCodes(ERROR_CODE_CRICITAL_BATTERY);
    }
}

//******************************************************************************
void LedsTask::requestNewLedGreenPattern(uint8_t count)
{
    requested_green_pattern_ = count;
    new_green_pattern_requested_ = true;
}
