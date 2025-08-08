/*
	MPU6050 Position Estimation with Raspberry Pi
*/

#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <math.h>

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

int fd;

/* Function to initialize MPU6050 */
void MPU6050_Init(){
	wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);  // Write to sample rate register
	wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);  // Write to power management register
	wiringPiI2CWriteReg8(fd, CONFIG, 0);         // Write to configuration register
	wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);   // Write to gyro configuration register
	wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);  // Write to interrupt enable register
}

/* Function to read raw accelerometer data */
short read_raw_data(int addr){
	short high_byte, low_byte, value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr + 1);
	value = (high_byte << 8) | low_byte;  // Combine high and low bytes
	return value;
}

int main(){
	float Acc_x, Acc_y, Acc_z;
	float Ax, Ay, Az;
	float Vx = 0, Vy = 0, Vz = 0;  // Velocity components
	float Px = 0, Py = 0, Pz = 0;  // Position components
	float dt = 0.5;  // Time interval in seconds (500 ms)
	
	fd = wiringPiI2CSetup(Device_Address);  // Initializes I2C with device address
	MPU6050_Init();                         // Initializes MPU6050
	
	while(1)
	{
		/* Read raw values from accelerometer */
		Acc_x = read_raw_data(ACCEL_XOUT_H);
		Acc_y = read_raw_data(ACCEL_YOUT_H);
		Acc_z = read_raw_data(ACCEL_ZOUT_H);
		
		/* Convert raw accelerometer values to acceleration in g */
		Ax = Acc_x / 16384.0;  // X-axis acceleration in g
		Ay = Acc_y / 16384.0;  // Y-axis acceleration in g
		Az = Acc_z / 16384.0;  // Z-axis acceleration in g
		
		/* Convert g to m/s² (1 g = 9.81 m/s²) */
		Ax = Ax * 9.81;
		Ay = Ay * 9.81;
		Az = Az * 9.81;
		
		/* Integrate acceleration to get velocity (v = u + a * t) */
		Vx += Ax * dt;
		Vy += Ay * dt;
		Vz += Az * dt;
		
		/* Integrate velocity to get position (p = p0 + v * t) */
		Px += Vx * dt;
		Py += Vy * dt;
		Pz += Vz * dt;
		
		/* Print position values */
		printf("Position: Px=%.3f m, Py=%.3f m, Pz=%.3f m\n", Px, Py, Pz);
		
		/* Delay to match the sampling rate (500 ms) */
		delay(500);
	}
	return 0;
}

