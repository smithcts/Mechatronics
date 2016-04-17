// Define constants used by globs.  Named enums are typedef to guarantee size.

#ifndef GLOB_CONSTANTS_H_INCLUDED
#define GLOB_CONSTANTS_H_INCLUDED

//******************************************************************************
typedef uint8_t glo_robot_command_t;
enum
{
    ROBOT_COMMAND_START,
    ROBOT_COMMAND_STOP,
    ROBOT_COMMAND_RESET,
    ROBOT_COMMAND_TIME_TASKS,
};

//******************************************************************************
typedef uint8_t glo_operating_state_t;
enum
{
    STATE_STOPPED,
    STATE_INITIALIZING,
    STATE_NORMAL
};

//******************************************************************************
typedef uint8_t glo_main_modes_t;
enum
{
    MAIN_MODE_BALANCE,
    MAIN_MODE_HORIZONTAL,
    MAIN_MODE_LINE_FOLLOWING,
    MAIN_MODE_EXPERIMENT,
    MAIN_MODE_CUSTOM,
    MAIN_MODE_RACE,

    NUM_MAIN_MODES
};

//******************************************************************************
typedef uint8_t glo_experiment_sub_modes_t;
enum
{
    EXPERIMENT_NONE_SELECTED,
    EXPERIMENT_MOTOR_SPEED_CONTROL,
    EXPERIMENT_MOTOR_POSITION_CONTROL,
    EXPERIMENT_NOT_IMPLEMENTED,

    NUM_EXPERIMENTS
};

//******************************************************************************
typedef uint8_t glo_error_codes_t;
enum
{
    ERROR_CODE_CRICITAL_BATTERY = 1,
};

//******************************************************************************
typedef uint8_t glo_wave_type_t;
enum
{
    WAVE_SINE,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_TRAPEZOIDAL,
    WAVE_CONSTANT
};

//******************************************************************************
typedef uint8_t glo_wave_state_t;
enum
{
    WAVE_STOPPED,      // Either no valid wave or wave is finished running.
    WAVE_READY_TO_RUN, // New wave received, but isn't running yet.
    WAVE_STARTING_UP,  // Brief time before wave actually starts where wave value is a constant 0.
    WAVE_STARTED,      // Wave is running normally.
};

//******************************************************************************
typedef uint32_t driving_command_id_t;
enum
{
    DRIVING_COMMAND_FORWARD        = 1,
    DRIVING_COMMAND_REVERSE        = 2,
    DRIVING_COMMAND_RIGHT          = 4,
    DRIVING_COMMAND_LEFT           = 8,
    DRIVING_COMMAND_STOP           = 16,
    DRIVING_COMMAND_DIRECT_COMMAND = 32,
};

//******************************************************************************
typedef uint8_t glo_pid_id_t;
enum
{
    PID_ID_LEFT_SPEED_CONTROLLER,
    PID_ID_RIGHT_SPEED_CONTROLLER,
    PID_ID_YAW_CONTROLLER,
    PID_ID_BALANCE_TILT_CONTROLLER,
    PID_ID_BALANCE_POSITION_CONTROLLER,
    PID_ID_LINE_TRACK_CONTROLLER,
    PID_ID_LEFT_POSITION_CONTROLLER,
    PID_ID_RIGHT_POSITION_CONTROLLER,

    NUM_PID_CONTROLLERS
};

//******************************************************************************
enum
{
    TELEMETRY_TEXT_SIZE = 200
};

enum maze_mode_t
{
    TRACK_LINE,
    ADVANCE,
    TURN_LEFT,
    TURN_RIGHT,
    TERMINATION,
    TURN_AROUND,
    CENTER
};

enum turn_mode_t
{
    LEFT,
    RIGHT,
    AROUND,
    END
};

#endif // GLOB_CONSTANTS_H_INCLUDED
