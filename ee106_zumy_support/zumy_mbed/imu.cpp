#include "mbed.h"
#include "imu.h"
#include "MPU6050.h" //for some reason, this cannot be included in imu.h.  I don't know why, but i put it here it works so ok.
#include "mbed_rpc.h"
#include "rtos.h"

DigitalOut imu_good(LED2);
MPU6050 mpu6050;
bool imu_ready= false; //variable that is set to true once init is done.
float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z; //init the floats for the RPC.
//the RPC variables for reading by zumys.
RPCVariable<float> rpc_accel_x(&accel_x, "accel_x");
RPCVariable<float> rpc_accel_y(&accel_y, "accel_y");
RPCVariable<float> rpc_accel_z(&accel_z, "accel_z");
RPCVariable<float> rpc_gryo_x(&gyro_x, "gyro_x");
RPCVariable<float> rpc_gryo_y(&gyro_y, "gyro_y");
RPCVariable<float> rpc_gryo_z(&gyro_z, "gyro_z");

//the RtosTimer that updates the IMU at ... 10 Hz sound ok?
RtosTimer* imu_timer;
int imu_time_ms = 100; //run every 100ms

void update_imu_wrapper(void const *n) //wrapper to leave function handle to update imu unchanged but still call it via an Rtos timer.
{
    update_imu();
}

void init_imu()
{
	uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
    
    if (whoami == 0x68) // WHO_AM_I should always be 0x68
    {
        mpu6050.MPU6050SelfTest(SelfTest);
        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
            mpu6050.initMPU6050();
            mpu6050.getAres();
            mpu6050.getGres();
            imu_ready = true;
            imu_good = 1;

            static RtosTimer imu_timer_(update_imu_wrapper, osTimerPeriodic);
            imu_timer = &imu_timer_;
            imu_timer->start(imu_time_ms);
        }
    }

}

void update_imu()
{
    if (imu_ready) {
        
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
            mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
            mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values

            // Now we'll calculate the accleration value into actual g's
            accel_x = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            accel_y = (float)accelCount[1]*aRes - accelBias[1];   
            accel_z = (float)accelCount[2]*aRes - accelBias[2];  
           
            // Calculate the gyro value into actual degrees per second
            gyro_x = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gyro_y = (float)gyroCount[1]*gRes - gyroBias[1];  
            gyro_z = (float)gyroCount[2]*gRes - gyroBias[2];
        }
    }
}

