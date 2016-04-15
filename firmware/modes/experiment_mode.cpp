// Includes
#include <cmath>
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "trigtables.h"
#include "util_assert.h"

// Forward declarations
static void updateWaveState(glo_wave_t & wave);
static void updateWave(glo_wave_t & wave, float delta_t);

//******************************************************************************
void MainControlTask::experimentMode(void)
{
    updateWaveState(wave_);

    updateWave(wave_, delta_t_);

    switch (modes_.sub_mode)
    {
        case EXPERIMENT_MOTOR_SPEED_CONTROL:
            experiment1Mode(wave_.value);
            break;
        case EXPERIMENT_MOTOR_POSITION_CONTROL:
            experiment2Mode(wave_.value);
            break;
        case EXPERIMENT_NOT_IMPLEMENTED:
            experiment3Mode(wave_.value);
            break;
    }
}

//******************************************************************************
static void updateWaveState(glo_wave_t & wave)
{
    // Time flag to determine when to fully start wave.
    static double wave_startup_end_time = 0;

    if (wave.state == WAVE_READY_TO_RUN)
    {
        glo_capture_command_t capture_command;
        glo_capture_command.read(&capture_command);
        if (capture_command.is_start && capture_command.paused)
        {
            capture_command.paused = false;
            main_control_task.handle(capture_command);
        }

        wave_startup_end_time = sys_timer.seconds();
        if ((wave.type != WAVE_CONSTANT) && (wave.frequency > 0))
        {
            wave_startup_end_time += (1.0f / wave.frequency) / 8.0f;
        }

        wave.state = WAVE_STARTING_UP;
    }

    if (wave.state == WAVE_STARTING_UP)
    {
        if (sys_timer.seconds() >= wave_startup_end_time)
        {
            wave.state = WAVE_STARTED;
        }
    }
}

//******************************************************************************
static void updateWave(glo_wave_t & wave, float delta_t)
{
    if (wave.state != WAVE_STARTED)
    {
        wave.value = 0;
        return;
    }

    float wave_period = 0;
    if (wave.frequency != 0)
    {
        wave_period = 1.0f / wave.frequency;
    }

    wave.time += delta_t;
    wave.total_time += delta_t;

    if ((wave.total_time > wave.duration) && !wave.run_continuous)
    {
        wave.value = 0;
        wave.state = WAVE_STOPPED;
        return;
    }

    // TODO - why not trapezoidal?
    if ((wave.time >= wave_period) && (wave.type != WAVE_TRAPEZOIDAL))
    {
        wave.time -= wave_period;
    }

    switch (wave.type)
    {
        case WAVE_SINE:
            float junk;
            sincostab(wave.time * wave.frequency * 2*PI, &wave.value, &junk);
            wave.value = wave.offset + wave.magnitude * wave.value;
            break;
        case WAVE_SQUARE:
            if ((wave.time < 0.0f) || (wave.time >= 0.5f * wave_period))
            {
                wave.value = wave.offset;
            }
            else
            {
                wave.value = wave.offset + wave.magnitude;
            }
            break;
        case WAVE_TRIANGLE:
            if (wave.time < 0.25f * wave_period)
            {
                wave.value = wave.offset + 4.0f * wave.magnitude * wave.frequency * wave.time;
            }
            else if (wave.time < 0.75f * wave_period)
            {
                wave.value = wave.offset + 2.0f * wave.magnitude * (1.0f - 2.0f * wave.frequency * wave.time);
            }
            else
            {
                wave.value = wave.offset + 4.0f * wave.magnitude * (wave.frequency * wave.time - 1.0f);
            }
            break;
        case WAVE_TRAPEZOIDAL:
            if (wave.time < 0)
            {
                wave.value = wave.offset;
            }
            else if (wave.time <= wave.t1) // the first spline function
            {
                wave.value = wave.offset + wave.c1[0]*(wave.time*wave.time) + wave.c1[1]*wave.time + wave.c1[2];
            }
            else if ((wave.time > wave.t1) && (wave.time <= wave.t2)) // the second spline function
            {
                wave.value = wave.offset + wave.c2[0]*(wave.time*wave.time) + wave.c2[1]*wave.time + wave.c2[2];
            }
            else if ((wave.time > wave.t2) && (wave.time <= wave.t3)) // the third spline function
            {
                wave.value = wave.offset + wave.c3[0]*(wave.time*wave.time) + wave.c3[1]*wave.time + wave.c3[2];
            }
            if (wave.time > wave.t3)
            {
                wave.value = wave.offset + wave.dx;
            }
            break;
        case WAVE_CONSTANT:
            wave.value = wave.offset;
            break;
        default:
            wave.value = 0;
            break;
    }

}

