// Includes
#include "tb6612fng.h"

// Shared between TB6612FNG class and timer update ISR
// The digital output pins control the direction of the motors
static DigitalOut a_in1(PE14);
static DigitalOut a_in2(PE13);
static DigitalOut b_in1(PE15);
static DigitalOut b_in2(PB10);

//*****************************************************************************
TB6612FNG::TB6612FNG(void)
{
    // Digital output pins should already be off, but just make sure.
    a_in1.clear();
    a_in2.clear();
    b_in1.clear();
    b_in2.clear();
}

//*****************************************************************************
void TB6612FNG::setDutyA(float duty)
{
    if (duty > 0.0f)
    {
        pwm_timer_.setDuty(duty, 2);
        a_in2.set();
        a_in1.clear();
    }
    else
    {
        pwm_timer_.setDuty(-duty, 2);
        a_in1.set();
        a_in2.clear();
    }
}

//*****************************************************************************
void TB6612FNG::setDutyB(float duty)
{
    if (duty > 0.0f)
    {
        pwm_timer_.setDuty(duty, 1);
        b_in2.set();
        b_in1.clear();
    }
    else
    {
        pwm_timer_.setDuty(-duty, 1);
        b_in1.set();
        b_in2.clear();
    }
}
