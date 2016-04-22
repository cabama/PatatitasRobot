
// REALIZAMOS LOS INCLUDES CORRESPONDIENTES
#include <stdio.h>
#include <stdint.h>
#include <iostream>			
#include <unistd.h> 		// Para sleep
#include <wiringPi.h>		// Para controlar el GPIO de Raspbian
#include <wiringPiI2C.h>	// Para controlar el I2C de Raspbian


// DIRECCIONES DE LOS REGISTROS I2C
// Direccion I2C
#define PCA9685_I2C_ADDR	0x40
// Direccion de las configuraciones
#define	MODE1	0x00
#define	MODE2	0x01
// Direcciones
#define SUBADDR1	0x02
#define SUBADDR2	0x03
#define SUBADDR3	0x04
#define ALLCALLADR	0x05
// Direcciones de los diferentes PWM (este PWM0)x4xN
#define LED0_ON_L	0x06
#define LED0_ON_H	0x07
#define LED0_OFF_L	0x08
#define LED0_OFF_H	0x09
// Direccion del Prescaler
#define PRE_SCALE	0x254
#define TESTMODE	0x255

// MODE 1 VALORES
#define MODE1_RESTART 
#define MODE1_SLEEP	 0x10

// Funciones introducidas en el precompilador para convertir int a uin8 low and
#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))



// CARGAMOS LA LIBRERIA STD (COUT CIN)
using namespace std;

int i2c_pwm = 0;


/*
 * Funcion Restart -> restablece los parametros por default del dispositivo PCA9685.
 * Para ello se debe modificar el bit RESTART del registro mode1 a 1
 */
 void restart()
 {
 	// Leemos el registro MODE1
    int mode1 = wiringPiI2CReadReg8 (i2c_pwm, MODE1);
    // Cambiamos el bit SLEEP a 1 del registro MODE 1
    uint8_t new_mode1 = mode1 | MODE1_SLEEP;
    // Escribimos estos cambios en el registro
    wiringPiI2CWriteReg8 (i2c_pwm, MODE1, new_mode1);
 }


/*
 * Ponemos el bit SLEEP del registro del mode1 a 1. (default)
 * Con esto no se puede utilizar el pwm pero permite cambiar la frecuencia del prescaler
 */

void mode1_sleep()
{
	// Leemos el registro MODE1
    int mode1 = wiringPiI2CReadReg8 (i2c_pwm, MODE1);
    // Cambiamos el bit SLEEP a 1 del registro MODE 1
    uint8_t new_mode1 = mode1 | MODE1_SLEEP;
    // Escribimos estos cambios en el registro
    wiringPiI2CWriteReg8 (i2c_pwm, MODE1, new_mode1);
}


/*
 * Ponemos el bit SLEEP del registro del mode1 a 0.
 * Con esto despertamos el PWM, pero no se puede cambiar el valor del prescaler.
 */
void mode1_wake_up()
{
    // Leemos el registro MODE1
    int mode1 = wiringPiI2CReadReg8 (i2c_pwm, MODE1);
    // Cambiamos el bit SLEEP a 0 del registro MODE 1
    uint8_t new_mode1 = mode1 & ~MODE1_SLEEP;
    // Escribimos estos cambios en el registro
    wiringPiI2CWriteReg8 (i2c_pwm, MODE1, new_mode1);
}


/*
 * Esta funcion cambia el valor del prescaler para poner el ciclo del pwm al valor deseado.
 *
 *  ENTRADA:
 *  Introducir la frecuencia del PWM en Hercios a la que se quiere el ciclo del PWM
 *
 * El calculo de la frecuencia del prescaler se encuentra en la página 25 del datasheet
 * La frecuencia minima del prescaler es de 24 Hz
 * La frecuencia maxima del prescaler es de 1526 Hz
 *
 * Writes to PRE_SCALE register are blocked when SLEEP bit is logic 0 (MODE 1) [DATASHEET].
 * Esto significa que tenemos que poner el bit SLEEP del mode1 a 1, funcion sleep antes de cambiar el valor.
 */

void set_frequency(int frecuencia)
{
	// Calculamos el valor del prescaler
	int osc_clock = 25000000;
	int bit12 = 4096;
	int prescale_value = (osc_clock/(bit12*frecuencia))-1;
	// Ponemos el bit sleep a 1 del registro Mode 1 para poder realizar cambiamos
	mode1_sleep();
	// Ahora si, cambiamos el registro del prescaler por el valor calculado
	wiringPiI2CWriteReg8 (i2c_pwm, PRE_SCALE, frecuencia);
	// Esperamos un determinado tiempo para salir del modo SLEEP
	usleep(5000);
	// Salimos del modo Sleep
	mode1_wake_up();
}


/*
 * Esta funcion cambia la señal de pwm para cada uno de los 16 pwm.
 *
 * PARAMETROS:
 * channel: pwm a modificar [0-15].
 * ON: valor de [0-4095] que indica cuando la señal en ON comienza.
 * OFF: valor de [0-4095] más grande que ON que indica cuando la señal baja. 
 */

void set_pwm_offset(int channel, uint16_t ON, uint16_t OFF)
{
	// Calculamos los cuatro bytes a escribir.
	uint8_t on_h = HIGHBYTE(ON);
	uint8_t on_l = LOWBYTE(ON);
	uint8_t off_h = HIGHBYTE(OFF);
	uint8_t off_l = LOWBYTE(OFF);

	// Escribimos en I2C
	int valor0 =  wiringPiI2CWriteReg8 (i2c_pwm, LED0_ON_L + 4 * channel, on_l);
	int valor1 =  wiringPiI2CWriteReg8 (i2c_pwm, LED0_ON_H + 4 * channel, on_h);
	int valor2 =  wiringPiI2CWriteReg8 (i2c_pwm, LED0_OFF_L + 4 * channel, off_l);
	int valor3 =  wiringPiI2CWriteReg8 (i2c_pwm, LED0_OFF_H + 4 * channel, off_h);
}



// Comenzamos en el MAIN realizando una conversion LOW and HIGH byte
int main(int argc, char *argv[])
{

	// Comunicacion con PCA9685
	i2c_pwm = wiringPiI2CSetup(PCA9685_I2C_ADDR);
    if (i2c_pwm == -1){
    	printf("No se encontro modulo I2C\n");
        return 0;
    }

    // Set frequencia
    set_frequency(200);
    set_pwm_offset(1,0,2043);

    usleep(5000000);

    set_frequency(1500);

    usleep(5000000);

    set_pwm_offset(1,0,1000);

    //set_pwm_offset(2,0,2043);

    return 0;
}