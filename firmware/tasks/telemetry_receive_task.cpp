// Includes
#include "telemetry_receive_task.h"
#include "globs.h"
#include "main_control_task.h"
#include "math_util.h"
#include "modes_task.h"
#include "robot_settings.h"
#include "telemetry_send_task.h"
#include "util_assert.h"

//******************************************************************************
TelemetryReceiveTask::TelemetryReceiveTask(void) :
      Task("Receive", TASK_ID_TELEM_RECEIVE),
      glo_rx_link_(NULL),
      bus_(USART_BUS_2),
      serial_port_(NULL)
{
}

//******************************************************************************
void TelemetryReceiveTask::initialize(void)
{
    // Setup receive link.
    // Make sure interrupts are disabled for initialization because the
    // first dma_rx error occurs before the fields the dma_rx are populated.
    bool enabled = scheduler.disableInterrupts();

    serial_port_ = Usart::instance(bus_);
    assert(serial_port_ != NULL, ASSERT_STOP);

    scheduler.restoreInterrupts(enabled);

    glo_rx_link_ = new GloRxLink(serial_port_, newMessageCallback);
    assert(glo_rx_link_ != NULL, ASSERT_STOP);

    syncPidParameters();
}

//*****************************************************************************
bool TelemetryReceiveTask::needToRun(void)
{
    return glo_rx_link_->dataReady();
}

//******************************************************************************
void TelemetryReceiveTask::run(void)
{
    // Parse any received data to try to form a complete message.
    // If a message is received then it is immediately handled in new message callback.
    // This will only parse one byte to make sure task returns quickly.
    glo_rx_link_->parse();
}

//******************************************************************************
void TelemetryReceiveTask::newMessageCallback(uint8_t object_id, uint16_t instance, void * glob_data)
{
    // Received a new glob over the serial port so need to decide how to handle it.
    switch (object_id)
    {
        case GLO_ID_DRIVING_COMMAND:
            receive_task.handle(*((glo_driving_command_t *)glob_data));
            break;
        case GLO_ID_CAPTURE_COMMAND:
            main_control_task.handle(*((glo_capture_command_t *)glob_data));
            break;
        case GLO_ID_MODES:
            modes_task.handle(*((glo_modes_t *)glob_data));
            break;
        case GLO_ID_ROBOT_COMMAND:
            modes_task.handle(*((glo_robot_command_t *)glob_data));
            break;
        case GLO_ID_WAVE:
            main_control_task.handle(*((glo_wave_t *)glob_data));
            break;
        case GLO_ID_PID_PARAMS:
            receive_task.handle(*((glo_pid_params_t *)glob_data), instance);
            break;
        case GLO_ID_REQUEST:
            receive_task.handle(*((glo_request_t *)glob_data), instance);
            break;
        default:
            assert_always_msg(ASSERT_CONTINUE, "Received unhandled glob with id: %d", object_id);
            break;
    }
}

//******************************************************************************
void TelemetryReceiveTask::handle(glo_driving_command_t & driving_command)
{
    glo_driving_command.publish(&driving_command);

    // Update motion commands using new driving command.
    glo_motion_commands_t motion_commands;
    glo_motion_commands.read(&motion_commands);

    if (driving_command.movement_type & DRIVING_COMMAND_RIGHT)
    {
        motion_commands.angular_velocity = -90.0f * DEG2RAD;
    }
    else if (driving_command.movement_type & DRIVING_COMMAND_LEFT)
    {
        motion_commands.angular_velocity = 90.0f * DEG2RAD;
    }
    else
    {
        motion_commands.angular_velocity = 0;
    }

    if (driving_command.movement_type & DRIVING_COMMAND_FORWARD)
    {
        if (motion_commands.linear_velocity > -0.00001f)
        {
            motion_commands.linear_velocity += 0.04f;
            limit(motion_commands.linear_velocity, -2.0f, 2.0f);
        }
        else // user hit forward to stop backing up
        {
            motion_commands.linear_velocity = 0.0f;
        }
    }
    else if (driving_command.movement_type & DRIVING_COMMAND_REVERSE)
    {
        if (motion_commands.linear_velocity < 0.00001f)
        {
            motion_commands.linear_velocity -= 0.04f;
            limit(motion_commands.linear_velocity, -2.0f, 2.0f);
        }
        else // user hit back to stop driving forward
        {
            motion_commands.linear_velocity = 0.0f;
        }
    }

    if (driving_command.movement_type & DRIVING_COMMAND_STOP)
    {
        motion_commands.linear_velocity = 0.0f;
        motion_commands.angular_velocity = 0.0f;
    }

    if (driving_command.movement_type & DRIVING_COMMAND_DIRECT_COMMAND)
    {
        // Scale down since app scale only does so much, switch sign on yaw to account
        // for coordinate change.  Easier to do here than on app.
        motion_commands.linear_velocity = driving_command.linear_velocity / 3.0f;
        motion_commands.angular_velocity = -driving_command.angular_velocity / 1.0f;
    }

    this->handle(motion_commands);
}

//******************************************************************************
void TelemetryReceiveTask::handle(glo_motion_commands_t & motion_commands)
{
    glo_motion_commands.publish(&motion_commands);
}

//******************************************************************************
void TelemetryReceiveTask::handle(glo_pid_params_t & params, uint16_t instance)
{
    uint16_t controller_id = instance-1;

    switch (controller_id)
    {
        case PID_ID_LEFT_SPEED_CONTROLLER:
            main_control_task.left_speed_pid.set(params);
            break;
        case PID_ID_RIGHT_SPEED_CONTROLLER:
            main_control_task.right_speed_pid.set(params);
            break;
        case PID_ID_YAW_CONTROLLER:
            main_control_task.yaw_pid.set(params);
            break;
        case PID_ID_BALANCE_TILT_CONTROLLER:
            main_control_task.handle_balance_tilt_gains(params);
            break;
        case PID_ID_BALANCE_POSITION_CONTROLLER:
            main_control_task.handle_balance_position_gains(params);
            break;
        case PID_ID_LINE_TRACK_CONTROLLER:
            main_control_task.line_track_pid.set(params);
            break;
        case PID_ID_LEFT_POSITION_CONTROLLER:
            main_control_task.left_position_pid.set(params);
            break;
        case PID_ID_RIGHT_POSITION_CONTROLLER:
            main_control_task.right_position_pid.set(params);
            break;
        default:
            assert_always_msg(ASSERT_CONTINUE, "No PID controller with ID %d", (int)controller_id);
    }

    glo_pid_params.publish(&params, instance);
}

//******************************************************************************
void TelemetryReceiveTask::syncPidParameters(void)
{
    glo_pid_params_t params[NUM_PID_CONTROLLERS];

    main_control_task.left_speed_pid.get(params[PID_ID_LEFT_SPEED_CONTROLLER]);
    main_control_task.right_speed_pid.get(params[PID_ID_RIGHT_SPEED_CONTROLLER]);
    main_control_task.yaw_pid.get(params[PID_ID_YAW_CONTROLLER]);
    main_control_task.get_balance_tilt_gains(params[PID_ID_BALANCE_TILT_CONTROLLER]);
    main_control_task.get_balance_position_gains(params[PID_ID_BALANCE_POSITION_CONTROLLER]);
    main_control_task.line_track_pid.get(params[PID_ID_LINE_TRACK_CONTROLLER]);
    main_control_task.left_position_pid.get(params[PID_ID_LEFT_POSITION_CONTROLLER]);
    main_control_task.right_position_pid.get(params[PID_ID_RIGHT_POSITION_CONTROLLER]);

    for (uint16_t i = 0; i < NUM_PID_CONTROLLERS; ++i)
    {
        glo_pid_params.publish(&params[i], i+1);
    }
}

//******************************************************************************
void TelemetryReceiveTask::handle(glo_request_t & msg, uint16_t instance)
{
    if ((instance == 0) && (msg.requested_id < NUM_GLOBS))
    {
        if (msg.requested_id == GLO_ID_ASSERT_MESSAGE)
        {
            send_task.send_cached_assert_messages();
        }
        else if (msg.requested_id == GLO_ID_DEBUG_MESSAGE)
        {
            send_task.send_cached_debug_messages();
        }
        else
        {
            // Send back all instances
            send_task.send(msg.requested_id, 1, globs[msg.requested_id]->get_num_instances());
        }
    }
    else
    {
        // Just send back the one instance that was requested.
        send_task.send(msg.requested_id, instance);
    }
}
