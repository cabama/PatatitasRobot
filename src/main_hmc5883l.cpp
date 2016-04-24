/*
 * Clase para el control de PCA9685 en ROS - PWM I2C de 16 Canales
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */


// INCLUIMOS LAS LIBRERIAS
#include <iostream>
#include <unistd.h>
#include <signal.h> // Para parar con control-c
#include <stdlib.h>
#include <stdio.h>
#include "drivers/hmc5883l.c"

// Para parar cuando llegue una señal ctrl-c
volatile sig_atomic_t ctrlc = 0;
volatile bool whiler = true;
void my_function(int sig){ // can be called asynchronously
  ctrlc = 1; // set flag
}


// FUNCION MAIN
int main () {

    // Parar con control-c
    signal(SIGINT, my_function);

    // Declaramos las dos estructuras y la clase acelerometro para su control
    CompassMsg mensaje;
    HMC5883L brujula;

    // Conectamos con el acelerometro y pedimos datos
    brujula.conectamos_brujula();


    while (true){
        if(ctrlc){
            whiler = false;
        }

        mensaje = brujula.getData();
        printf("Magnetometro [x, y, z] = [%d, %d, %d] - Angulo = %f \n", mensaje.x, mensaje.y, mensaje.z, mensaje.angle);
    }


}