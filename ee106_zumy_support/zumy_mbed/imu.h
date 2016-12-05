#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"


extern bool imu_ready; //extern because it's actually created in imu.cpp
extern float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z; //init the floats for the RPC.

void init_imu(); //initialize the IMU
void update_imu();  //update the IMU RPC variables.
void update_imu_wrapper(void const *n); //wrapper so an RTOS timer inside imu.cpp can call update_Imu.



#endif
