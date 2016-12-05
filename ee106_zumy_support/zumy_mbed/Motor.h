#ifndef MOTOR_H_
#define MOTOR_H_

#include "mbed.h"
#include "rtos.h"

class Motor {

private:

    //todo: drive-coast vs drive-brake
	PwmOut xout_1;
	PwmOut xout_2;
    
    float clamp(float input, float min, float max); //helper clamp functions
    

public:
    float pwm_val;
	void pwm_speed(float dutyCycle); //public only for emergency circumstances, please don't use regularly....
    Motor(PinName xout_1_in, PinName xout_2_in);
    void brake(float value);
    bool is_inverted;  //deep, deep down, reverses motor direction.
    //Useeful for drive trains where the bi-lateral symmetry across the robot means going positive on both motors results in the robot spinning.
};

#endif
