#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "hmc5883l.h"

#define PI 3.14159265

int HMC5883L::conectamos_brujula(){

  // Setup Wiring Pi
  wiringPiSetup();
  // Open an I2C connection
  id_i2c = wiringPiI2CSetup(HMC5883L_ADDRESS);
  // Perform I2C work
  wiringPiI2CWriteReg8(id_i2c, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);

}

CompassMsg HMC5883L::get_data(){

  // OBTENEMOS EL CAMPO MAGNETICO EN LOS TRES EJES
  // Eje x
  uint8_t msb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_MSB_X);
  uint8_t lsb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_LSB_X);
  short x = msb << 8 | lsb;
  // Eje y
  msb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_MSB_Y);
  lsb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_LSB_Y);
  short y = msb << 8 | lsb;
  // Eje z
  msb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_MSB_Z);
  lsb = wiringPiI2CReadReg8(id_i2c, HMC5883L_REG_LSB_Z);
  short z = msb << 8 | lsb;
  // Obtenemos el angulo en el plano X
  float angle = atan2((float) y, (float)x) * (180 / PI);
  // Empaquetamos en la estructura
  CompassMsg msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;
  msg.angle = angle;

  return msg;

}

/*
  // Obtenemos el angulo en el plano X
  float angle = atan2((float) y, (float) x);
  if(angle < 0)
    angle += 2*PI;
  if(angle > 2*PI)
    angle -= 2*PI;
  angle = angle * 180/M_PI; 
*/
