/*
 * Clase para el control de GY-521 mediante I2C.
 * GY-521 - Acelerometro // Giroscopio 3 ejes (x, y, z)
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */


/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *	INCLUIMOS LAS LIBRERIAS NECESARIAS
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */
#ifndef PATATITAS_GY_521_H
#define PATATITAS_GY_521_H

#include <stdio.h>
#include <stdint.h>
#include <iostream>         // Utilizacion de la clase STD
#include <unistd.h> 		// Para sleep
#include <math.h> 			// Para realizar el atan2
#include <wiringPi.h>		// Para controlar el GPIO de Raspbian
#include <wiringPiI2C.h>	// Para controlar el I2C de Raspbian


/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *	DEFINIMOS EN EL PRECOMPILADOR LOS VALORES DE LAS DIRECCIONES
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */
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


/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *	DEFINIMOS EL STRUCT
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */

struct Espacio
{
    int16_t x;
    int16_t y;
    int16_t z;
};


/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *	DEFINIMOS LA CLASE
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */

class MPU6050{
private:
    int id_i2c;
public:
    void conectamos_acelerometro();
    Espacio get_aceleraciones();
    Espacio get_giroscopio();
};

#endif //PATATITAS_GY_521_H