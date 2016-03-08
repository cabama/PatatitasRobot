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
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
// Registros del acelerometro 
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R

// VARIABLES GLOBALES
int fd;

// Creamos una structura de espacio el cual servira para almacenar datos {x,y,z}
struct Espacio
{
        int16_t x;
        int16_t y;
        int16_t z;
};


// Nos conectamos al Acelerometro
void conectamos_acelerometro(){
    fd = wiringPiI2CSetup(MPU6050_I2C_ADDRESS);
    wiringPiI2CReadReg8 (fd, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);
}

// Funcion para obtener las aceleraciones
Espacio get_aceleraciones(){

	Espacio aceleraciones; // Generamos una estructura {x,y,z} para almacenar aceleraciones

	// Obtenemos la aceleracion en x
    uint8_t ax1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_XOUT_H);
    uint8_t ax2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_XOUT_L);
    aceleraciones.x = (ax1 << 8) | ax2;

    // Obtenemos la aceleracion en y
    uint8_t ay1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_YOUT_H);
    uint8_t ay2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_YOUT_L);
    aceleraciones.y = (ay1 << 8) | ay2;

    // Obtenemos la aceleracion en z
    uint8_t az1 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_ZOUT_H);
    uint8_t az2 = wiringPiI2CReadReg8(fd, MPU6050_ACCEL_ZOUT_L);
	aceleraciones.z = (az1 << 8) | az2;

	return aceleraciones;

}

// Funcion para obtener el giroscopio
Espacio get_giroscopio(){

	Espacio giroscopio; // Generamos una estructura {x,y,z} para almacenar aceleraciones

	// Obtenemos la aceleracion en x
    uint8_t ax1 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_XOUT_H);
    uint8_t ax2 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_XOUT_L);
    giroscopio.x = (ax1 << 8) | ax2;

    // Obtenemos la aceleracion en y
    uint8_t ay1 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_YOUT_H);
    uint8_t ay2 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_YOUT_L);
    giroscopio.y = (ay1 << 8) | ay2;

    // Obtenemos la aceleracion en z
    uint8_t az1 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_ZOUT_H);
    uint8_t az2 = wiringPiI2CReadReg8(fd, MPU6050_GYRO_ZOUT_L);
	giroscopio.z = (az1 << 8) | az2;

	return giroscopio;
}



// Funcion main
// int main()
//{
//
//	conectamos_acelerometro();
//	Espacio aceleraciones = get_aceleraciones();
//	Espacio giroscopios   = get_giroscopio();
//
//	printf("Funcion conectada? fd = %d \n", fd);
//	printf("Aceleraciones: x=%d, y=%d, z=%d \n", aceleraciones.x, aceleraciones.y, aceleraciones.z);
//	printf("Giroscopio: x=%d, y=%d, z=%d \n", giroscopios.x, giroscopios.y, giroscopios.z);
//
//    return 0;
//}