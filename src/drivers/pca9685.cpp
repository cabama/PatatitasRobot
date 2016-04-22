/*
 * Clase para el control de PCA9685 - PWM I2C de 16 Canales
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */


// REALIZAMOS LOS INCLUDES CORRESPONDIENTES
#include <stdio.h>
#include <stdint.h>
#include <iostream>         
#include <unistd.h>         // Para sleep
#include <wiringPi.h>       // Para controlar el GPIO de Raspbian
#include <wiringPiI2C.h>    // Para controlar el I2C de Raspbian



/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *  DEFINIMOS EN EL PRECOMPILADOR LOS VALORES DE LAS DIRECCIONES
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */


// DIRECCIONES DE LOS REGISTROS I2C
#define PCA9685_I2C_ADDR    0x40 // Direccion I2C, modificar si se ha cambiado.
// Direccion de las configuraciones
#define MODE1       0x00
#define MODE2       0x01
// Direcciones
#define SUBADDR1    0x02
#define SUBADDR2    0x03
#define SUBADDR3    0x04
#define ALLCALLADR  0x05
// Direcciones de los diferentes PWM (este PWM0)x4xN
#define LED0_ON_L   0x06
#define LED0_ON_H   0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09
// Direccion del Prescaler
#define PRE_SCALE   254
#define TESTMODE    255

// MODE 1 VALORES
#define MODE1_RESTART   0x80
#define MODE1_SLEEP     0x10

// Funciones introducidas en el precompilador para convertir int a uin8 low and
#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

// Numero de unidades por ciclo PWM
#define CICLO_PWM 4095 // Un ciclo PWM tiene doce bits




/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *  DECLARAMOS CLASE PWM
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */


/*
 * Declaramos la Clase PWM que se encargara del control de PCA9685
 */

class PCA9685{
public:

    // Declaramos las variables globales
    int i2c_pwm;    // Variable global que contiene el identifador del I2C - Wiring Pi

    // Declaramos las funciones
    PCA9685();                                                      // Constructor de la clase
    void restart();                                                 // Reinicia el chip
    void mode1_sleep();                                             // Pone el chip en modo sleep
    void mode1_wake_up();                                           // Despierta el chip
    void set_frequency(int frecuencia);                             // Establece la frecuencia del reloj
    void set_pwm_offset(int channel, uint16_t ON, uint16_t OFF);    // Establece un canal PWM [por flancos]
    void set_pwm_tanto(int channel, int tanto_on);                  // Establece un canal PWM [tanto por ciento]
    int  print_mode1();

};




/*
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 *  METODOS DE LA CLASE PWM
 *  = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 */


/*
 * CONSTRUCTOR DE LA CLASE
 */
PCA9685::PCA9685()
{
    // Comunicacion con PCA9685
    i2c_pwm = wiringPiI2CSetup(PCA9685_I2C_ADDR);
    if (i2c_pwm == -1){
        printf("No se encontro el dispositivo en esta direccion I2C: %d \n", PCA9685_I2C_ADDR);
    }
}


/*
 * Funcion Restart -> restablece los parametros por default del dispositivo PCA9685.
 * Para ello se debe modificar el bit RESTART del registro mode1 a 1
 */
 void PCA9685::restart()
 {
    // Leemos el registro MODE1
    int mode1 = wiringPiI2CReadReg8 (i2c_pwm, MODE1);
    usleep(50000);
    uint8_t new_mode1 = mode1 | MODE1_RESTART;
    // Escribimos estos cambios en el registro
    wiringPiI2CWriteReg8 (i2c_pwm, MODE1, MODE1_RESTART);

 }


/*
 * Ponemos el bit SLEEP del registro del mode1 a 1. (default)
 * Con esto no se puede utilizar el pwm pero permite cambiar la frecuencia del prescaler
 */

void PCA9685::mode1_sleep()
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
void PCA9685::mode1_wake_up()
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

void PCA9685::set_frequency(int frecuencia)
{
    // Calculamos el valor del prescaler
    int osc_clock = 25000000;
    int prescale_value = (osc_clock/(CICLO_PWM*frecuencia))-1;
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

void PCA9685::set_pwm_offset(int channel, uint16_t ON, uint16_t OFF)
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


/*
 * Esta funcion establece el ciclo pwm de los 16 canales.
 * A difrencia de la función anterior, no se considera un offset inicial.
 *
 * PARAMETROS:
 * Channel: canal del pwm ha modificar [0-15]
 * Tanto: Tanto por ciento que se desea establecer la señal en ON [0-100]
 */

void PCA9685::set_pwm_tanto(int channel, int tanto_on) {

    // Calculamos los flancos de ON.
    uint16_t ON = 0;
    uint16_t OFF = CICLO_PWM * tanto_on / 100;

    // Llamamos a la funcion original con los datos calculados.
    set_pwm_offset(channel,ON,OFF);
}

int PCA9685::print_mode1()
{
    return wiringPiI2CReadReg8 (i2c_pwm, MODE1);
}