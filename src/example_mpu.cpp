/*
 * Clase para el control de PCA9685 en ROS - PWM I2C de 16 Canales
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */


// INCLUIMOS LAS LIBRERIAS
#include <iostream>
#include <unistd.h>
#include "drivers/GY-521.cpp"


// FUNCION MAIN
int main () {

    // Declaramos las dos estructuras y la clase acelerometro para su control
    Espacio aceleracion, gyroscocion;
    MPU6050 acelerometro;

    // Conectamos con el acelerometro y pedimos datos
    acelerometro.conectamos_acelerometro();
    aceleracion = acelerometro.get_aceleraciones();
    gyroscocion = acelerometro.get_giroscopio();

    // Pedimos datos por pantalla
    printf("Aceleracion: [%d, %d, %d]\n", aceleracion.x, aceleracion.y, aceleracion.z);
    printf("Giroscopo: [%d, %d, %d]\n", gyroscocion.x, gyroscocion.y, gyroscocion.z);

}