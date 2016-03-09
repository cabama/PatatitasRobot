/*
 * Clase para el control de PCA9685 en ROS - PWM I2C de 16 Canales
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */

#include <iostream>
#include <unistd.h> 		// Para sleep
#include "drivers/pca9685.cpp"


int main(){

    printf("Hola Mundo\n");
    PCA9685 Pwm;
    //Pwm.restart();
    //Pwm.mode1_wake_up();
    Pwm.set_frequency(1000);
    usleep(500);
    //Pwm.mode1_sleep();
    printf("Mode1: %d",Pwm.print_mode1());

    int motor, velocidad;

    while (true) {

        printf("\nIntroduzca el motor a mover [-1 == reset]: ");
        std::cin >> motor;

        if (motor >= 0 ) {
            printf("\nIntroduzca una velocidad: ");
            std::cin >> velocidad;
            Pwm.set_pwm_offset((int) motor, 0, (int) velocidad);
        }


    }


    return 0;
}


