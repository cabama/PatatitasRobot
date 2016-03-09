/*
 * Clase para el control de GY-521 mediante I2C.
 * GY-521 - Acelerometro // Giroscopio 3 ejes (x, y, z)
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */
#include "GY-521.h"


/*
 * Metodo para conectar con el acelerometro por I2C.
 * Es nesario ser invocado al comienzo para poder tener el control de GY521.
 * Asigna un valor a id_i2c para su control i2c con wiringPi hace de (fd).
 * Pensar si en el futuro convertirla en constructor.
 */
void MPU6050::conectamos_acelerometro(){
    id_i2c = wiringPiI2CSetup(MPU6050_I2C_ADDRESS);
    wiringPiI2CReadReg8 (id_i2c, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(id_i2c, MPU6050_PWR_MGMT_1, 0);
}


/*
 * Esta funcion devuelve la aceleracion en los tres ejes.
 * Lo devuelve en la Estructura Espacio que contiene 3 int [x, y, z]
 */
Espacio MPU6050::get_aceleraciones() {

    Espacio aceleraciones; // Generamos una estructura {x,y,z} para almacenar aceleraciones

    // Obtenemos la aceleracion en x
    uint8_t ax1 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_XOUT_H);
    uint8_t ax2 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_XOUT_L);
    aceleraciones.x = (ax1 << 8) | ax2;

    // Obtenemos la aceleracion en y
    uint8_t ay1 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_YOUT_H);
    uint8_t ay2 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_YOUT_L);
    aceleraciones.y = (ay1 << 8) | ay2;

    // Obtenemos la aceleracion en z
    uint8_t az1 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_ZOUT_H);
    uint8_t az2 = wiringPiI2CReadReg8(id_i2c, MPU6050_ACCEL_ZOUT_L);
    aceleraciones.z = (az1 << 8) | az2;

    return aceleraciones;
}


/*
 * Esta funcion devuelve la aceleracion angular en los tres ejes.
 * Lo devuelve en la Estructura Espacio que contiene 3 int [x, y, z]
 */
Espacio MPU6050::get_giroscopio(){

    Espacio giroscopio; // Generamos una estructura {x,y,z} para almacenar aceleraciones

    // Obtenemos la aceleracion en x
    uint8_t ax1 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_XOUT_H);
    uint8_t ax2 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_XOUT_L);
    giroscopio.x = (ax1 << 8) | ax2;

    // Obtenemos la aceleracion en y
    uint8_t ay1 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_YOUT_H);
    uint8_t ay2 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_YOUT_L);
    giroscopio.y = (ay1 << 8) | ay2;

    // Obtenemos la aceleracion en z
    uint8_t az1 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_ZOUT_H);
    uint8_t az2 = wiringPiI2CReadReg8(id_i2c, MPU6050_GYRO_ZOUT_L);
    giroscopio.z = (az1 << 8) | az2;

    return giroscopio;
}