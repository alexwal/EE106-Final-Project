#include "mbed.h"
#include "mbed_rpc.h"

#include "QEI.h"
#include "MODSERIAL.h"
#include "rtos.h"
#include "pindefs.h"
#include "Track.h"
#include "imu.h"
#include "comms.h"


//various ints and floats that are updated by the mbed that are read over RPC.
int r_enc, l_enc;
float r_spd,l_spd;

RPCVariable<int>   rpc_r_enc(&r_enc, "r_enc");
RPCVariable<int>   rpc_l_enc(&l_enc, "l_enc");
RPCVariable<float> rpc_r_spd(&r_spd, "r_spd");
RPCVariable<float> rpc_l_spd(&l_spd, "l_spd");


DigitalOut init_done(LED1);
DigitalOut main_loop(LED3);
DigitalOut test(LED4);

Track* track_right;
Track* track_left;




void led_blink_periodic(void const *args) {
    // Toggle the green LED when this function is called.
    test = !test;
}

int main() {


    RtosTimer ledBlinkTimer(led_blink_periodic);
    ledBlinkTimer.start(200);

   

    init_comms();

    Track track_left_ = Track(MOTOR_1_1,MOTOR_1_2,p29,p30,624);
    Track track_right_ = Track(MOTOR_2_1,MOTOR_2_2,p11,p12,624); //these both need to be instantiaed inside main, not statically above.
    track_left = &track_left_;
    track_right = &track_right_;
    track_right -> invert(true); //I start inverted.
    pc.printf("Tracks initialized \n\r");

    init_imu();  //also starts an rtos thread for the purpose of reading the IMU.
    pc.printf("IMU is ready is %d \n\r",imu_ready);

    // receive commands, and send back the responses

    while(1) 
    {
        // Handle the encoders, used for testing if rpc variable reads work
        r_enc=track_right->get_position();
        l_enc=track_left->get_position();
        r_spd=track_right->get_speed();
        l_spd=track_left->get_speed();

        Thread::wait(10);  //non-blocking wait.  may not be 100% precise, but better than blocking.
    }

}


void enable(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_enable(&enable,"enable");
void enable(Arguments* input, Reply *output) //0 to disable tracks.  anything else to enable.  Put other commands here, as neccessary, to disable the robot as appropriate.
{
    int arg0 = input->getArg<int>();
    bool enab = false;
    if((arg0) > 0)
    {
        enab = true;    
    }
    
    track_left->enable(enab);
    track_right->enable(enab);

}


void sm(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_sm(&sm, "sm");
void sm(Arguments* input, Reply *output)  //SetMotors.  Currently, just sets velocity setpoint.  We can work on bringing back open-loop options later.
{

    float arg0 = input->getArg<float>();
    float arg1 = input->getArg<float>();

    track_left -> set_auto(true);
    track_right -> set_auto(true);


    track_left->set_velocity_setpoint(arg0);
    track_right->set_velocity_setpoint(arg1);

}


void pid(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_pid(&pid, "pid");
void pid(Arguments* input, Reply *output)
{
    //function that sets the track's PID settings.  Useful for chaing the PID gains if the zumy is carrying an additional load.

    float kp = input->getArg<float>();
    float ki = input->getArg<float>();
    float kd = input->getArg<float>();

    track_right->set_gains(kp,ki,kd);
    track_left->set_gains(kp,ki,kd);
}
