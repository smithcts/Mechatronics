// Includes
#include <cmath>
#include "globs.h"
#include "main_control_task.h"
#include "physical_constants.h"
#include "robot_settings.h"
#include "util_assert.h"

//******************************************************************************
void MainControlTask::experiment3Mode(float experiment_input)
{
    if (throttle(2))
    {
        assert_always_msg(ASSERT_CONTINUE, "This experiment isn't implemented");
    }
}
