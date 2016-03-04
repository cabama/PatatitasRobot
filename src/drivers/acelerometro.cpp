/*
 *	GY-521 - Acelerometro // Giroscopio 3 ejes (x, y, z)
 */

#include <stdio.h>
#include <stdint.h>
#include <iostream>			
#include <unistd.h> 		// Para sleep
#include <math.h> 			// Para realizar el atan2
#include <wiringPi.h>		// Para controlar el GPIO de Raspbian
#include <wiringPiI2C.h>	// Para controlar el I2C de Raspbian

// Direcciones de I2C
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_I2C_ADDRESS        0x68   // I2C
// Registros del giroscopio
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_H        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_XOUT_H        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_XOUT_H        0x48   // R
// Registros del acelerometro 
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R

 int main()
{
    int fd = wiringPiI2CSetup(MPU6050_I2C_ADDRESS);
    if (fd == -1)
        return 0;
 
    wiringPiI2CReadReg8 (fd, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);
 
    //float gx,gy,gz;
 
    while(true)
    {
        //gx = wiringPiI2CReadReg8(fd, MPU6050_GYRO_XOUT_H);
        //gy = wiringPiI2CReadReg8(fd, MPU6050_GYRO_YOUT_H);
        //gz = wiringPiI2CReadReg8(fd, MPU6050_GYRO_ZOUT_H);

        uint8_t ax1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_XOUT_H);
        uint8_t ax2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_XOUT_L);
        int16_t ax = (ax1 << 8) | ax2;

        uint8_t ay1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_YOUT_H);
        uint8_t ay2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_YOUT_L);
        int16_t ay = (ay1 << 8) | ay2;

        uint8_t az1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_ZOUT_H);
        uint8_t az2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_ZOUT_L);
		int16_t az = (az1 << 8) | az2;

 		//printf("Giroscopio:\n");
        //printf("x=%f   y=%f   z=%f \n", gx,gy,gz);
        printf("Acelerometro:\n");
        printf("x=%d   y=%d   z=%d \n", ax,ay,az);
        printf("\n");

	    usleep(50000);
	    
    } // Fin del while

    return 0;
}