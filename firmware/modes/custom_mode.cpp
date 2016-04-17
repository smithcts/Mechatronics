// Includes
#include "globs.h"
#include "physical_constants.h"
#include "main_control_task.h"
#include "leds_task.h"
#include "debug_printf.h"
#include "pid_controller.h"

#define QTR_THRESHOLD (0.5f)
#define NOMINAL_SPEED (0.3f)




void determine_direction(float dir, int8_t scale, float divisor);
float determine_line_pos(uint8_t qtr_state, float line_posistion, uint8_t num_qtr_on);
float start_distance = 0.0f;
float absolute_beginning_distance = 0.0f;
float delta_distance;
float distance_between_node = 0.0f;
float node_beginning_distance = 0.0f;
float distance_traveled = 0.0f;
float delta_yaw = 0.0f;
int8_t scaler_yaw = 0;
float Xdirection, Ydirection;
float INCREMENTAL_SPEED = 0.0f;
float TEST = 0.0f;
float prev_distance = 0.0f;
float node_distance;

//******************************************************************************
void MainControlTask::customMode(void)
{
    if (modes_.state == STATE_NORMAL)
    {
        int8_t qtr_state = 0;
        uint8_t num_qtr_on = 0;
        static int8_t cnt = 0;
        static float line_position;

        static maze_mode_t maze_mode = TRACK_LINE;
        static turn_mode_t turn_mode = LEFT;

        static float yaw;
        static float current_yaw;
        static float direction_yaw;
        static float distanceX, distanceY;



        float delta_speed = 0.0f, left_speed_command = 0.0f, right_speed_command = 0.0f;

        absolute_beginning_distance = odometry_.avg_distance;

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

        TEST = odometry_.avg_distance;

        switch (maze_mode)
        {
        case TRACK_LINE:

            delta_speed = track_maze_line_pid.calculate(0.0f - line_position, delta_t_);

            delta_distance = node_distance - prev_distance;
            INCREMENTAL_SPEED = INCREMENTAL_SPEED + 0.05;

            if(INCREMENTAL_SPEED > NOMINAL_SPEED)
            {
                INCREMENTAL_SPEED = NOMINAL_SPEED;
            }

            current_yaw = odometry_.yaw;
            direction_yaw =(current_yaw - yaw) * 1000 ;

            if(num_qtr_on > 3)
            {

                if (cnt < 10)
                {
                    left_speed_command = NOMINAL_SPEED/4 + delta_speed;
                    right_speed_command = NOMINAL_SPEED/4 - delta_speed;
                }
                else
                {
                    if((analog_.voltages[0] > QTR_THRESHOLD) && (analog_.voltages[7] > QTR_THRESHOLD))
                    {
                        start_distance = odometry_.avg_distance;
                        turn_mode = LEFT;
                        maze_mode = ADVANCE;
//                       debug_printf("node found");
                    }
                    else if((analog_.voltages[7] > QTR_THRESHOLD) && (analog_.voltages[0] < QTR_THRESHOLD))
                    {
                        start_distance = odometry_.avg_distance;
                        turn_mode = LEFT;
                        maze_mode = ADVANCE;
//                        debug_printf("left turn");
                    }
                    else if((analog_.voltages[0] > QTR_THRESHOLD) && (turn_mode != LEFT) && (analog_.voltages[7] < QTR_THRESHOLD))
                    {
                        start_distance = odometry_.avg_distance;
                        turn_mode = RIGHT;
                        //                  maze_mode = ADVANCE;
//                        debug_printf("right turn");
                    }

                }
                cnt++;
            }
            else if (num_qtr_on == 0)
            {
                start_distance = odometry_.avg_distance;
                turn_mode = AROUND;
                maze_mode = ADVANCE;
            }
            else if((turn_mode == END))
            {
                maze_mode = TERMINATION;
                debug_printf("TERMINATION");
            }


            left_speed_command = INCREMENTAL_SPEED + delta_speed;
            right_speed_command = INCREMENTAL_SPEED - delta_speed;
            break;

        case ADVANCE:

            left_speed_command = INCREMENTAL_SPEED/2 + delta_speed;
            right_speed_command = INCREMENTAL_SPEED/2 - delta_speed;

            if ((odometry_.avg_distance - start_distance) > 0.135f)
            {
                yaw = odometry_.yaw;
                switch(turn_mode)
                {
                case AROUND:

                    maze_mode = TURN_AROUND;
                    scaler_yaw += 2;
                    determine_direction(TEST, scaler_yaw, 2.0f);
//                    debug_printf("Turn Around: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    break;

                case LEFT:
                    maze_mode = TURN_LEFT;
                    scaler_yaw -= 1;
                    determine_direction(TEST, scaler_yaw, 1.0f);
//                    debug_printf("Turn Left: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    break;

                case RIGHT:
                    if(num_qtr_on == 0)
                    {
                        maze_mode = TURN_RIGHT;
                        scaler_yaw += 1;
                        determine_direction(TEST, scaler_yaw, 1.0f);
//                      debug_printf("Turn Right: Direction %i Scaler %i\n", (int)direction_yaw, scaler_yaw);
                    }
                    else scaler_yaw += 0;
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
            node_distance = odometry_.avg_distance;

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
            node_distance = odometry_.avg_distance;
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
            node_distance = odometry_.avg_distance;

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
        prev_distance = delta_distance;

        motor_pwm_.left_duty = left_duty_command;
        motor_pwm_.right_duty = right_duty_command;

    }
}

void determine_direction(float dir, int8_t scale, float divisor)
{
    static float old_distance = 0.0f;
    float distance = (dir - old_distance);
    if(scale < 0)
    {

        scale = scale * -1;


        if (scale % 2 == 0)
        {
            //even but negative, south, y-direction

            Ydirection -= dir;
            debug_printf("South: DeltaY %i scaler %i Y %i\n", (int)(1000*dir), (int)(scale * -1), (int)(1000*distance));
        }
        else
        {
            //odd but negative, west, x-direction
            Xdirection -= dir;
            debug_printf("West: DeltaX %i scaler %i X %i\n", (int)(1000*dir), (int)(scale * -1), (int)(1000*distance));
        }
    }
    else
    {


        if(scale % 2 == 0)
        {
            //even, positive, north, y-direction
            Ydirection += dir;
            debug_printf("North: DeltaY %i scaler %i Y %i\n", (int)(1000*dir), (int)(scale), (int)(1000*distance));
        }
        else
        {
            //odd, positive, east, x-direction
            Xdirection += dir;
            debug_printf("East: DeltaX %i scaler %i X %i\n", (int)(1000*dir), (int)(scale), (int)(1000*distance));
        }
    }
    old_distance = distance;
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
