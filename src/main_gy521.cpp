/*
 * Clase para el control de acelerometro
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 * 
 * Descripcion:
 * Este fichero se conecta a ROS y utiliza la libreria/driver que se encuentra en ./driver/GY-5221.cpp
 */


// INCLUIMOS LAS LIBRERIAS
#include <iostream>
#include <unistd.h>
#include <signal.h> // Para parar con control-c
#include <stdlib.h>
#include <stdio.h>
#include "drivers/GY-521.cpp"

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
    Espacio aceleracion, gyroscocion;
    MPU6050 acelerometro;

    // Conectamos con el acelerometro y pedimos datos
    acelerometro.conectamos_acelerometro();


    while (true){
        if(ctrlc){
            whiler = false;
        }
        aceleracion = acelerometro.get_aceleraciones();
        gyroscocion = acelerometro.get_giroscopio();
        printf("Aceleracion: [%d, %d, %d]\n", aceleracion.x, aceleracion.y, aceleracion.z);
        printf("Giroscopo: [%d, %d, %d]\n", gyroscocion.x, gyroscocion.y, gyroscocion.z); 
    }


}