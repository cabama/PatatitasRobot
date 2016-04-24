/*
 * Clase para el control de PCA9685 en ROS - PWM I2C de 16 Canales
 * Autor: Carlos Barreiro Mata
 * Email: barreymata@gmail.com
 */

#include <iostream>
#include <unistd.h>  // Para sleep
#include "drivers/pca9685.cpp"


int main(){

    printf("Hola Mundo\n");
    PCA9685 Pwm;
    Pwm.restart();
    //Pwm.mode1_wake_up();
    Pwm.set_frequency(1000);
    usleep(500);
    //Pwm.mode1_sleep();
    printf("Mode1: %d",Pwm.print_mode1());

    int motor, velocidad;

    bool ejecucion = true;

    while (ejecucion) {

        printf("\nIntroduzca el motor a mover\n [-5 wakeup][-4 sleep] [-3 mode1] [-2 reset] [-1 Finish]: ");
        std::cin >> motor;

        if (motor >= 0 ) {
            printf("\nIntroduzca una velocidad: ");
            std::cin >> velocidad;
            Pwm.set_pwm_offset((int) motor, 0, (int) velocidad);
        }

        else if (motor == -2){
            printf("\n RESTAURAR LA CLASE");
            Pwm.restart();
        }

        else if (motor == -3){
            int mode1 = Pwm.print_mode1();
            printf("MODE 1: %d", mode1);
        }

        else if (motor == -4){
            Pwm.mode1_sleep();
        }

        else if (motor == -5){
            Pwm.mode1_wake_up();
        }

        else if (motor == -1)
            ejecucion = false;

    }


    return 0;
}


