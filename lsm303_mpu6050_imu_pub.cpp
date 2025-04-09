#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <pigpiod_if2.h>
#include <iostream>

const int LSM303_accel=0x19; //accelerometer address
const int MPU6050_gyro  = 0x68;

const int I2Cbus = 1;
const int MPU_ADDR = 0x68;
const int PWR_MGMT_1   = 0x6B;
const int SMPLRT_DIV   = 0x19;
const int CONFIG       = 0x1A;
const int GYRO_CONFIG  = 0x1B;
const int INT_ENABLE   = 0x38;
const int ACCEL_XOUT_H = 0x3B;
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_ZOUT_H = 0x3F;
const int GYRO_XOUT_H  = 0x43;
const int GYRO_YOUT_H  = 0x45;
const int GYRO_ZOUT_H  = 0x47;


const float RAD_PER_DEG = 0.0174533;

int pi = -1;
int ACCEL_HANDLE=-1;
int GYRO_HANDLE=-1;

sensor_msgs::Imu myImu;

using namespace std;

int pigpio_setup();
void get_gyro();
void gyro_setup();
void fast_write(unsigned int reg, unsigned int val);
void accel_setup();
void get_accel();


int main(int argc, char **argv) {
        ros::init(argc, argv, "imu_publisher");
        ros::NodeHandle node;
        ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 0);
        int pi = pigpio_setup();
        if (pi >= 0) {
                cout << "daemon interface started ok at " << pi << endl;
        }
        else {
                cout << "Failed to connect to PIGPIO daemon - is it running?" << endl;
                return -1;
        }
        accel_setup();
        gyro_setup();
        myImu.header.frame_id = "imu";
        myImu.orientation_covariance[0] = -1;
        for (int i = 0; i < 9; ++i) {
                myImu.linear_acceleration_covariance[i]=0;
                myImu.angular_velocity_covariance[i]=0;
        }

        ros::Rate loop_rate(10);
        while(ros::ok())
        {
                get_accel();
                get_gyro();
                pub.publish(myImu);
                loop_rate.sleep();
        }
	i2c_write_byte_data(pi, ACCEL_HANDLE, 0x20, 0x00);//puts accelerometer into sleep mode
        i2c_close(pi, ACCEL_HANDLE);
        i2c_write_byte_data(pi, GYRO_HANDLE, 0x6B, 0);//puts gyro to sleep mode
        i2c_close(pi, GYRO_HANDLE);

        return 1;
}


int pigpio_setup() {
        char* addr_str = NULL;
        char* port_str = NULL;
        pi = pigpio_start(addr_str, port_str);
        return pi;
}

void gyro_setup() {
        GYRO_HANDLE = i2c_open(pi, I2Cbus, MPU_ADDR, 0);
        if (GYRO_HANDLE >= 0) {
                cout << "Gyro found. Handle = " << GYRO_HANDLE << endl;
        }
        else {
                cout<<"Unable to open I2C comms with Gyro"<<endl;
        }
        time_sleep(0.05);
        fast_write(SMPLRT_DIV, 7);
        fast_write(PWR_MGMT_1, 1);
        fast_write(CONFIG, 0);
}

void fast_write(unsigned int reg, unsigned int val){
        i2c_write_byte_data(pi, GYRO_HANDLE, reg, val);
        time_sleep(0.02);
}

void get_gyro() {
        int xLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_XOUT_H);
        int xMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_XOUT_H + 1);
        myImu.angular_velocity.x = (float)((int16_t)(xLSB << 8 | xMSB));
        int yLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_YOUT_H);
        int yMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_YOUT_H + 1);
        myImu.angular_velocity.y = (float)((int16_t)(yLSB << 8 | yMSB));
        int zLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_ZOUT_H);
        int zMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, GYRO_ZOUT_H + 1);
        myImu.angular_velocity.z = (float)((int16_t)(zLSB << 8 | zMSB));
}

void accel_setup()
{
    //initiate comms with accelerometer and get handle
    ACCEL_HANDLE=i2c_open(pi,I2Cbus, LSM303_accel,0);
    if (ACCEL_HANDLE>=0)
    {
        cout<<"Accelerometer found. Handle = "<<ACCEL_HANDLE<<endl;
    }
    else
    {
        cout<<"Unable to open I2C comms with Accelerometer"<<endl;
    }

    i2c_write_byte_data(pi, ACCEL_HANDLE, 0x20, 0x47); //set frequency
    time_sleep(.02);
    i2c_write_byte_data(pi, ACCEL_HANDLE, 0x23, 0x09); //continuous update, LSB at lower addr, +- 2g, Hi-Res disable
    time_sleep(.02);
}

void get_accel()
{
    int xLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x28);
    int xMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x29);
    //12 bits resolution, MSB right-jusified, then convert to Tesla
    myImu.linear_acceleration.x=(float)((int16_t)(xLSB | xMSB<<8)>>4)/1000*9.81;

    int yLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2A);
    int yMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2B);
    myImu.linear_acceleration.y=(float)((int16_t)(yLSB | yMSB<<8)>>4)/1000*9.81;

    int zLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2C);
    int zMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2D);
    myImu.linear_acceleration.z=(float)((int16_t)(zLSB | zMSB<<8)>>4)/1000*9.81;

    myImu.header.stamp = ros::Time::now();
}

