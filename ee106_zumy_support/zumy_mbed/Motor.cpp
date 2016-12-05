#include "mbed.h"
#include "Motor.h"

//a DRV_8833 motor driver, one of two halfs.

Motor::Motor(PinName xout_1_in,
             PinName xout_2_in
             )
    : xout_1(xout_1_in), xout_2(xout_2_in)
{
    xout_1.period_ms(3); //~300Hz, as specified by HW engineer.
    xout_2.period_ms(3);

    pwm_speed(0.0);
}


void Motor::pwm_speed(float value)
{
    float val = clamp(value,-1,1); //clamp between -1 and 1, just as a saftey measure.
    pwm_val = value;
    if(is_inverted)
    {
        val = -value;
    }
    else
    {
        val = value;
    }

    if(val == (float) 0.0f)
    {
        xout_1.write(0.0);
        xout_2.write(0.0);
    }
    else if(val > 0.0f) //let forward mean that it's AH,BL
    {
        xout_1.write(val);
        xout_2.write(0.0);
    }
    else//let backward mean that it's Al,BH
    {
        xout_1.write(0.0);
        xout_2.write(-val);
        //negative because value is less than 1 here.
    }
}

void Motor::brake(float value) //Shorts motor across GND.  using truth table of PG 9 of the datasheet
{
    float val = clamp(value,0,1); //clamp it to be between zero and 1.  take the abs just in case...
    xout_1.write(val);
    xout_2.write(val);
}



float Motor::clamp(float input, float min, float max)
{
    if( input > max)
    {
        return max;
    }
    else if (input < min)
    {
        return min;
    }
    else
    {
        return input;
    }

}
