// Includes
#include "globs.h"
#include "physical_constants.h"
#include "main_control_task.h"
#include "leds_task.h"
#include "debug_printf.h"
#include "pid_controller.h"

#define QTR_THRESHOLD (0.5f)
#define NOMINAL_SPEED (0.4f)





float determine_line_pos(uint8_t qtr_state, float line_posistion, uint8_t num_qtr_on);
float start_distance = 0.0f;
float absolute_beginning_distance = 0.0f;
float delta_distance = 0.0f;
float distance_between_node = 0.0f;
float node_beginning_distance = 0.0f;
float distance_traveled = 0.0f;
float delta_yaw = 0.0f;
int8_t scaler_yaw = 0;

//******************************************************************************
void MainControlTask::customMode(void)
{
    if (modes_.state == STATE_NORMAL)
    {
        int8_t qtr_state = 0;
        uint8_t num_qtr_on = 0;
        static float INCREMENTAL_SPEED = 0.0f;
        static int8_t cnt = 0;
        static int8_t initcnt = 0;
        static float line_position;

        static maze_mode_t maze_mode = TRACK_LINE;
        static turn_mode_t turn_mode = LEFT;

        static float yaw;
        static float current_yaw;
        static float direction_yaw;
        static float distanceX, distanceY;


        float delta_speed = 0.0f, left_speed_command = 0.0f, right_speed_command = 0.0f;

        for(uint8_t i = 0; i < 8; i ++)
        {
            if (analog_.voltages[i] > QTR_THRESHOLD)
            {
                qtr_state |= i<<i;
                num_qtr_on++;
            }
        }

        leds_task.requestNewLedGreenPattern(qtr_state);
        line_position = determine_line_pos(qtr_state, line_position, num_qtr_on);

        switch (maze_mode)
        {
        case TRACK_LINE:

            delta_speed = track_maze_line_pid.calculate(0.0f - line_position, delta_t_);
            INCREMENTAL_SPEED = NOMINAL_SPEED;
            absolute_beginning_distance = odometry_.avg_distance;

            distanceX = absolute_beginning_distance - start_distance;
            delta_distance = absolute_beginning_distance - start_distance;
            distanceX = distanceX * 100;

            current_yaw = odometry_.yaw;
            direction_yaw =(current_yaw - yaw) * 1000 ;

            if ((analog_.voltages[0] > QTR_THRESHOLD) || (analog_.voltages[7] > QTR_THRESHOLD) || (num_qtr_on > 3))
            {
                start_distance = odometry_.avg_distance;
                if (cnt < 10)
                {
                    left_speed_command = INCREMENTAL_SPEED/6 + delta_speed;
                    right_speed_command = INCREMENTAL_SPEED/6 - delta_speed;
                }
                else
                {
                    if((analog_.voltages[0] > QTR_THRESHOLD) && (analog_.voltages[7] > QTR_THRESHOLD))
                    {
                        turn_mode = LEFT;
                        maze_mode = ADVANCE;
                        debug_printf("node found");
                    }
                    else if((analog_.voltages[7] > QTR_THRESHOLD) &! (analog_.voltages[0] > QTR_THRESHOLD))
                    {
                        turn_mode = LEFT;
                        maze_mode = ADVANCE;
                        debug_printf("left turn");
                    }
                    else if((analog_.voltages[0] > QTR_THRESHOLD) &! (analog_.voltages[7] > QTR_THRESHOLD))
                    {
                        turn_mode = RIGHT;
                        maze_mode = ADVANCE;
                        debug_printf("right turn");
                    }
                    else
                    {
                        debug_printf("straight");
                    }
                }
            }
            else if (num_qtr_on == 0)
            {
                start_distance = odometry_.avg_distance;
                turn_mode = AROUND;
                maze_mode = ADVANCE;
            }

            left_speed_command = INCREMENTAL_SPEED + delta_speed;
            right_speed_command = INCREMENTAL_SPEED - delta_speed;
            cnt++;
            break;

        case ADVANCE:

            distance_between_node = start_distance - node_beginning_distance;
            left_speed_command = INCREMENTAL_SPEED/2;
            right_speed_command = INCREMENTAL_SPEED/2;

            if ((odometry_.avg_distance - start_distance) > 0.135f)
            {
                yaw = odometry_.yaw;
                distance_traveled = odometry_.avg_distance - start_distance;
                switch(turn_mode)
                {
                case AROUND:
                    maze_mode = TURN_AROUND;
                    scaler_yaw -= 2;
                    debug_printf("Turn Around: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    break;

                case LEFT:
                    maze_mode = TURN_LEFT;
                    scaler_yaw += 1;
                    debug_printf("Turn Left: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    break;

                case RIGHT:
                    if(num_qtr_on == 0)
                    {
                        maze_mode = TURN_RIGHT;
                        scaler_yaw -= 1;
                        debug_printf("Turn Right: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    }
                    else
                    {
                        maze_mode = TURN_LEFT;
                    }
                    break;
                }
            }
            break;

        case TERMINATION:
            left_speed_command = 0.0f;
            right_speed_command = 0.0f;
            break;

        case TURN_AROUND:
            left_speed_command = INCREMENTAL_SPEED/2;
            right_speed_command = -INCREMENTAL_SPEED/2;

            if((analog_.voltages[3] > QTR_THRESHOLD) && (odometry_.yaw - yaw) < PI/8)
            {
                cnt = 0;
                INCREMENTAL_SPEED = 0;

                delta_yaw = odometry_.yaw - yaw;
                maze_mode = TRACK_LINE;

            }

            break;

        case TURN_LEFT:
            left_speed_command = -INCREMENTAL_SPEED/2;
            right_speed_command = INCREMENTAL_SPEED/2;

            if ((analog_.voltages[6] > QTR_THRESHOLD) && (odometry_.yaw - yaw) > PI/8)
            {
                cnt = 0;
                delta_yaw = odometry_.yaw - yaw;
                maze_mode = TRACK_LINE;
                INCREMENTAL_SPEED = 0;
//                   debug_printf("turn left");
            }
            break;

        case TURN_RIGHT:
            left_speed_command = INCREMENTAL_SPEED/2;
            right_speed_command = -INCREMENTAL_SPEED/2;

            if ((analog_.voltages[1] > QTR_THRESHOLD) && (odometry_.yaw - yaw) < PI/8)
            {
                cnt = 0;
                delta_yaw = odometry_.yaw - yaw;
                maze_mode = TRACK_LINE;
                INCREMENTAL_SPEED = 0;
//                   debug_printf("turn left");
            }
            break;
        }

        float left_duty_command = left_speed_pid.calculate(left_speed_command - odometry_.left_speed, delta_t_);
        float right_duty_command = right_speed_pid.calculate(right_speed_command - odometry_.right_speed, delta_t_);

        motor_pwm_.left_duty = left_duty_command;
        motor_pwm_.right_duty = right_duty_command;

    }
}

float determine_line_pos(uint8_t qtr_state, float last_line_pos, uint8_t num_qtr_on)
{
    static int8_t line_lost_counter = 0;
    float line_pos = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (qtr_state & (1<<i))
        {
            line_pos += (0.009525*i-0.0333375);
        }
    }
    if ((num_qtr_on) && (num_qtr_on < 3))
    {
        if (line_lost_counter) line_lost_counter -= 1;
        else line_pos = line_pos/num_qtr_on;
    }
    else
    {
        line_lost_counter = 9;
        line_pos = last_line_pos;
    }

    return (line_pos);
}
