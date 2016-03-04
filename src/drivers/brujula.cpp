#include <stdio.h>
#include <stdint.h>
#include <unistd.h> // Para sleep
#include <math.h> // Para realizar el atan2
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;


/*
 Autor: Carlos Barreiro Mata <barreymata@gmail.com>

 Datasheet
 https://www.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
 
 REGISTRO A:
 Valor por defecto: 16
 En este registro se configura:
 - Numero de muestras promedias (1 default)
 - la tasa de datos DO (15Hz default)
 - Modo de medicion

 REGISTRO B:
 Se modifica la ganancia

 REGISTRO MODE:
 Se configura el modo de salida de los bits (continuo o con lectura cuando pedimos)
 Ademas se puede establecer el I2C a alta velocidad. (desabilitado por defecto)
*/


// Direccion I2C por defecto de la brujula
const int direccion = 30;

// Direccion de los diferentes registros
const uint8_t registroA = 00;
const uint8_t registroB = 01;
const uint8_t registroMode = 02;
const uint8_t x_msb_reg = 3, x_lsb_reg = 4;
const uint8_t y_msb_reg = 7, y_lsb_reg = 8;
const uint8_t z_msb_reg = 5, z_lsb_reg = 6;


// Se predeclara la funcion consultar
short *consultar(int fd);




/* 
 *  MAIN DE LA FUNCION:
 *  Actualmente simplemente se leen y muestra por pantalla el estado de los registros de configuracion.
 *  Se inicia un bucle en el que se consulta 100 veces con una pausa de un segundo el estado de los 
 *  campos magneticos.
 */

int main()
{

   // Inicializamos la comunicación con la brujula
   int fd = wiringPiI2CSetup(direccion);
   if (fd==-1)
      return -1;


   // Consultamos los registros
   int regA =  wiringPiI2CReadReg8(fd, registroA) ;
   int regB =  wiringPiI2CReadReg8(fd, registroB) ;
   int regM =  wiringPiI2CReadReg8(fd, registroMode) ;
   // Imprimimos los registros
   printf("Hexadeicimal: %x, %d \n", 0x1e, 0x1e);
   printf("registroA: %x [%d]\n", regA, regA);
   printf("registroB: %x [%d]\n", regB, regB);
   printf("registroMode: %x [%d]\n", regM, regM);

   
   short *puntos;

   for(int i=0;i<100;i++)
   {
      puntos = consultar(fd);
      short x = *puntos, y = *(puntos+1); // z = *puntos+2;
      float angulo = atan2(y, x) * (180 / 3.14159)+180;
      printf("La orientacion es de: %.2f \n\n", angulo);
      printf("X: %x [%d]\n", x, x);
      printf("Y: %x [%d]\n", y, y);
      //printf("Z: %x [%d]\n", z, z);
      usleep(1000000);
   } 

   return 0; // Could be return(0)
}



/*
*  CONSULTAR:
*  Devuelve: un puntero con la direccion del array que contiene la informacion de los campos
*  magneticos, se encuentran ordenados de la siguiente forma {x,y,z}
*
*  Esta función lee los diferentes registros que contiene la informacion y junta los bits de
*  mayor y menor orden
*/

short *consultar (int fd)
{

   // Variables que guardan los resultados
   short *puntos = new short[3]; 

   // Consultamos los puntos de X
   uint8_t x_msb = wiringPiI2CReadReg8(fd, x_msb_reg); // Consultamos los bits de alto nivel
   uint8_t x_lsb = wiringPiI2CReadReg8(fd, x_lsb_reg); // Consultamos los bits de bajo nivel
   puntos[0] = (x_msb << 8) | x_lsb; // Juntamos los bits de alto y bajo nivel

   // Consultamos los puntos de Y
   uint8_t y_msb = wiringPiI2CReadReg8(fd, y_msb_reg); // Consultamos los bits de alto nivel
   uint8_t y_lsb = wiringPiI2CReadReg8(fd, y_lsb_reg); // Consultamos los bits de bajo nivel
   puntos[1] = (y_msb << 8) | y_lsb; // Juntamos los bits de alto y bajo nivel

   // Consultamos los puntos de Z
   uint8_t z_msb = wiringPiI2CReadReg8(fd, z_msb_reg); // Consultamos los bits de alto nivel
   uint8_t z_lsb = wiringPiI2CReadReg8(fd, z_lsb_reg); // Consultamos los bits de bajo nivel
   puntos[2] = (z_msb << 8) | z_lsb; // Juntamos los bits de alto y bajo nivel

   // Retornamos los puntos.
   return puntos;
}
