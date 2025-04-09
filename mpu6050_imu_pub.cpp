#include <pigpiod_if2.h>
#include <iostream>

using namespace std;

const int I2C_BUS = 1;
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

const float TESLA_PER_GAUSS = .0001;
const float RAD_PER_DEG = 0.0174533;

void fast_write(unsigned int reg, unsigned int val, int handle);
int read_raw_data(unsigned int addr, int handle);

char* addr_str = NULL;
char* port_str = NULL;
int pi = pigpio_start(addr_str, port_str);

int main() {
	if (pi >= 0) cout << "PIGPIO's daemon was successfuly started ok at " << pi << endl;
	else {
		cout << "Can't start the PIGPIO's daemon. Try 'sudo systemctl start pigpiod.service' or 'sudo pigpiod'" << endl;
		return -1;
	}
	const int MPU_HANDLE = i2c_open(pi, I2C_BUS, MPU_ADDR, 0);
	fast_write(SMPLRT_DIV, 7, MPU_HANDLE);
        fast_write(PWR_MGMT_1, 1, MPU_HANDLE);
        fast_write(CONFIG, 0, MPU_HANDLE);
        fast_write(GYRO_CONFIG, 24, MPU_HANDLE);
	fast_write(INT_ENABLE, 1, MPU_HANDLE);

	while (true) {
		int acc_x = read_raw_data(ACCEL_XOUT_H, MPU_HANDLE);
		int acc_y = read_raw_data(ACCEL_YOUT_H, MPU_HANDLE);
		int acc_z = read_raw_data(ACCEL_ZOUT_H, MPU_HANDLE);

		int gyro_x = read_raw_data(GYRO_XOUT_H, MPU_HANDLE);
		int gyro_y = read_raw_data(GYRO_YOUT_H, MPU_HANDLE);
		int gyro_z = read_raw_data(GYRO_ZOUT_H, MPU_HANDLE);

		int Ax = acc_x/16384.0;
		int Ay = acc_y/16384.0;
		int Az = acc_z/16384.0;

		int Gx = gyro_x/131.0;
		int Gy = gyro_y/131.0;
		int Gz = gyro_z/131.0;

		cout << "gx: " << gyro_x << " gy: " << gyro_y << " gz: " << gyro_z << endl;
	}
	return 0;
}

void fast_write(unsigned int reg, unsigned int val, int handle){
	i2c_write_byte_data(pi, handle, reg, val);
	time_sleep(0.02);
}

int read_raw_data(unsigned int addr, int handle) {
	int high = i2c_read_byte_data(pi, handle, addr);
	int low = i2c_read_byte_data(pi, handle, addr + 1);
	int value = ((high << 8) | low);
	if (value > 32768) value -= 65536;
	return value;
}
