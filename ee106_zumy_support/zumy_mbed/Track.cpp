#include "mbed.h"
#include "Track.h"


int avg_width = 5; //average over 10 cycles of the encoders.  Still, i'll do control on dTicks, not on the boxcar averaged data.
//low speeds: 400 ticks/sec.  

float encoder_read_rate = .006; //sec.  can't be too small or quantization kills you.  Need to eventually write timing-based QEI library that can perform speed estimation.

float pid_control_rate = 0.02; //sec
int PID_CONTROLL_RATE = 20; //need int number of ms for the rtos code.


Track::Track(PinName motor_1, PinName motor_2, PinName enc_A, PinName enc_B, int pulses_per_rev) : motor(motor_1,motor_2), enc(enc_A,enc_B,NC,pulses_per_rev), encoder_changes(avg_width,0), control_timer(&Track::wrap_execute_control, osTimerPeriodic, this), controller(1,2,0,pid_control_rate)
	//sorry for the massively long one-liner.  Don't know why it doesn't work when i break it up.
	//create a control timer, motor, encoder, encoder_changes, and PID controller all on construction
	//construct the track object. 
    //initial dTicks = 0\
    // Kp=1, Ki=2, kD = 0, period is PID controller.  Works decent for wheels-in-the-air zumy. Can't set P=0 initially, apparently.
	{

    old_position = 0; //I start at encoder equals zero.


    controller.setInputLimits(-3500,3500); // +/- 2700 pulses-sec, highest I've seen, with zumy's wheels in the air
    controller.setOutputLimits(-1,1); //PWM can go from -1 to 1
    controller.setBias(0); //except that I start at zero (stopped)
    controller.setMode(1);  //turn PID ON.
    controller.setInterval(pid_control_rate);

    set_auto(false);

    inverted = 1;
    enabled = false; //i start disabled, thank you very much.

    //now that i've constructed the timers, attack the correct function to the (already running) encoder ticker
    //and start the rtos Timer for the speed pid loop.
    encoder_timer.attach(this, &Track::execute_timeout,encoder_read_rate);
    control_timer.start(PID_CONTROLL_RATE);
}

void Track::manual_speed(float dutyCycle)
{
	set_auto(false);
	if(enabled)
	{
		motor.pwm_speed(inverted*dutyCycle);
	}
}
void Track::set_velocity_setpoint(float speed)
{
	closed_loop = true;
    controller.setSetPoint(speed);
}

int Track::get_position()
{
	return inverted*enc.getPulses();
}
float Track::get_speed()
{
	return encoder_changes.GetAverage()/encoder_read_rate; //divide by encoder read rate to recover ticks/sec.
}

void Track::set_gains(float kp, float ki, float kd)
{
	controller.setTunings(kp,ki,kd);
}

void Track::execute_timeout()
{
	//called every 2 ms.  open question if it should be an RTOS timer.
	int pos = inverted*enc.getPulses();
	encoder_changes.Insert(pos-old_position);
	old_position = pos;
}

void Track::invert(bool invert_state)
{
	motor.is_inverted = invert_state;
	if(invert_state) //i AM inverted
	{
		inverted = -1;
	}
	else  //i am NOT inverted
	{
		inverted = 1;
	}
}

void Track::set_auto(bool val)
{
    closed_loop = val;
    if(val)
    {
        controller.setMode(1);
    }
    else
    {
        controller.setMode(0);
        //zero is manual
    }
}


float Track::get_setpoint()
{
    return controller.getSetpoint();
}

void Track::wrap_execute_control(const void *track)
{
	//a super=magical line of code that i got from Doug's 192 implimentation, that he in turn got from Nikita.
    const_cast<Track*>(static_cast<const Track*>(track))->execute_control();
}

void Track::execute_control()  //function to execute closed loop control
{
    // Only change the speed if the motor is running in auto
    // (speed-controlled) mode
    if (closed_loop & enabled)  //don't compute if i'm disabled.  prevents integral wind-up.
    {
        controller.setProcessValue(get_speed());
        float val = controller.compute();
        //need inverted here  impirically determined.
        motor.pwm_speed(clamp(-inverted*val,-1,1));
    }
}

float Track::getPwm()
{
	return motor.pwm_val;
}

float Track::clamp(float input, float min, float max)
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

void Track::enable(bool state)
{
	enabled = state;
	if(state == false)
	{
		motor.pwm_speed(0); //turn off the motor if i'm disabled!
	}
}
