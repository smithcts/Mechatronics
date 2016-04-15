// Includes
#include "globs.h"
#include "leds_task.h"
#include "main_control_task.h"

//******************************************************************************
void MainControlTask::lineFollowingMode(void)
{
    // Nominal travel speed (m/s)
    float speed_command = .25;

    // Offset of line from center of QTR array (in meters).
    float line_position = 0;

    // How high voltage has to get before detecting line.
    const float line_thresh = 1.0f;

    // State of green LEDs to show what sensors is seeing line.
    uint8_t led_state = 0;

    // Distance between each IR sensors in meters.
    const float ir_spacing = 0.009525;

    // Total number of IR sensors seeing the line;
    uint8_t num_seeing_line = 0;

    // Measured line position from last call.
    static float last_line_position = 0;

    // Sum of sensor distances seeing line.
    float line_position_sum = 0;

    // Used to detect multiple lines.
    bool stopped_seeing = false;
    bool split_detected = false;

    // Use the QTR voltages to determine which ones are seeing the line.
    // The left most sensor is first element.
    const uint8_t num_sensors = 8;
    for (uint8_t i = 0; i < num_sensors; ++i)
    {
        bool seeing_line = (analog_.voltages[i] >= line_thresh);

        // Set status of current LED to be on only if seeing line.
        led_state |= seeing_line << i;

        if (seeing_line)
        {
            line_position_sum += i * ir_spacing;
            num_seeing_line++;
        }

        // Detect gaps in sensor measurements to avoid taking wrong path.
        if (!split_detected)
        {
            split_detected = stopped_seeing && seeing_line;
            stopped_seeing = !seeing_line && (num_seeing_line > 0);
        }
    }

    if (num_seeing_line == 0)
    {
        // Preserve last measurement so we can get back on line.
        line_position = last_line_position;
    }
    else if (split_detected)
    {
        // Use decaying last reading and hope that spit goes away. Don't just set to zero
        // since that will cause a large derivative spike which would turn us in the wrong direction.
        line_position = last_line_position / 2.0f;
    }
    else
    {
        // Find distance of line from left side of sensors. (meters)
        line_position = line_position_sum / num_seeing_line;

        // Make zero position be in center of array.
        line_position -= ((num_sensors-1) / 2.0f) * ir_spacing;
    }

    leds_task.requestNewLedGreenPattern(led_state);

    // Calculate difference in duty cycle between motors needed to track line.
    // Always command desired line position to zero.
    float delta_duty = line_track_pid.calculate(0.0f - line_position, delta_t_);

    float left_duty_command = left_speed_pid.calculate(speed_command - odometry_.left_speed, delta_t_);
    float right_duty_command = right_speed_pid.calculate(speed_command - odometry_.right_speed, delta_t_);

    if (modes_.state != STATE_NORMAL)
    {
        left_duty_command = 0.0f;
        right_duty_command = 0.0f;
        delta_duty = 0.0f;
        left_speed_pid.resetIntegral();
        right_speed_pid.resetIntegral();
        line_track_pid.resetIntegral();
    }

    motor_pwm_.left_duty = left_duty_command + delta_duty;
    motor_pwm_.right_duty = right_duty_command - delta_duty;

    // Save for next time
    last_line_position = line_position;

}
