/**
 * Subscriptor_gamepad.cpp
 * - - - - - - - - - - - -
 * 
 * Este nodo se encarga de subscribirse al topic del gamepad que se publica desde el otro PC.
 * A partir de los datos recibidos se aplican 
 * Desarrollado por patatitasTeam®
 * 
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * 
 * Estructura del mensaje GamePad. <std_msgs/Float32MultiArray>
 * El mensaje Gamepad se trata de un array tipo Float 32, ya que con los jostick tenemos decimales.
 * A continuacion detallaremos cada una de las posiciones que ocupa en el array cada uno de los botones.
 * 
 * [0]  -> Jostick izquierdo, eje x.
 * [1]  -> Jostick izquierdo, eje y.  (Arriba es -1 y abajo es +1)[El valor puede no llegar a 1 y quedarse en 0.9]
 * [2]  -> Jostick derecho, eje x.
 * [3]  -> Jostick derecho, eje y. (Arriba es -1 y abajo es +1)[El valor puede no llegar a 1 y quedarse en 0.9]
 * [4]  -> Triangulo.
 * [5]  -> Circulo.
 * [6]  -> Equis.
 * [7]  -> Cuadrado.
 * [8]  -> L1.
 * [9]  -> R1.
 * [10] -> L2.
 * [11] -> R2.
 * [12] -> Select.
 * [13] -> Start.
 * [14] -> L3.
 * [15] -> R3.
 *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * 
 * Conexiones de PWM.
 * 
 * - En el canal 0 se encuentra rueda izquierda marcha atras.
 * - En el canal 1 se encuentra rueda izquierda marcha adelante.
 * - En el canal 2 se encuentra rueda derecha   marcha adelante.
 * - En el canal 3 se encuentra rueda derecha   marcha atras.
 * 
 */


// LIBRERIAS.
#include <iostream>                 	// Libreria Standar
#include "ros/ros.h"					// Libreria ROS
#include "std_msgs/Float32MultiArray.h" // Libreria del Mensaje Float Array
#include "drivers/pca9685.cpp"			// Libreria de los drivers del PWM

// NAMESPACES
using namespace std;

// VARIABLES GLOBALES.
const string nodo_name = "gamepad_subscriptor";
const string topic_name = "gamepad";
PCA9685 Pwm;


/**
 * @brief Con esta funcion se accede a la maquina de estados para mover el robot con el funcionamiento de los gatillos L2 y R2.
 * 
 * @param L2 informacion del estado del boton L2 del gamepad
 * @param R2 informacion del estado del boton R2 del gamepad
 */
void programacion_gatillos(int L2, int R2)
{
    if (L2 == 1 && R2 == 0)
    {
    	Pwm.set_pwm_tanto(0, 0); 	// En el canal 0 se encuentra rueda izquierda marcha atras.
    	Pwm.set_pwm_tanto(1, 70); 	// En el canal 1 se encuentra rueda izquierda marcha adelante.
    	Pwm.set_pwm_tanto(2, 0); 	// En el canal 2 se encuentra rueda derecha   marcha adelante.
    	Pwm.set_pwm_tanto(3, 70); 	// En el canal 3 se encuentra rueda derecha   marcha atras.
    }
    else if (L2 == 0 && R2 == 1)
    {
    	Pwm.set_pwm_tanto(0, 70);	// En el canal 0 se encuentra rueda izquierda marcha atras.
    	Pwm.set_pwm_tanto(1, 0);	// En el canal 1 se encuentra rueda izquierda marcha adelante.
    	Pwm.set_pwm_tanto(2, 70);	// En el canal 2 se encuentra rueda derecha   marcha adelante.
    	Pwm.set_pwm_tanto(3, 0);	// En el canal 3 se encuentra rueda derecha   marcha atras.
    }
    else if (L2 == 1 && R2 == 1)
    {
    	Pwm.set_pwm_tanto(0, 0);	// En el canal 0 se encuentra rueda izquierda marcha atras.
    	Pwm.set_pwm_tanto(1, 70);	// En el canal 1 se encuentra rueda izquierda marcha adelante.
    	Pwm.set_pwm_tanto(2, 70);	// En el canal 2 se encuentra rueda derecha   marcha adelante.
    	Pwm.set_pwm_tanto(3, 0);	// En el canal 3 se encuentra rueda derecha   marcha atras.
    }
    else if (L2 == 0 && R2 == 0)
    {
    	Pwm.set_pwm_tanto(0, 0);	// En el canal 0 se encuentra rueda izquierda marcha atras.
    	Pwm.set_pwm_tanto(1, 0);	// En el canal 1 se encuentra rueda izquierda marcha adelante.
    	Pwm.set_pwm_tanto(2, 0);	// En el canal 2 se encuentra rueda derecha   marcha adelante.
    	Pwm.set_pwm_tanto(3, 0);	// En el canal 3 se encuentra rueda derecha   marcha atras.
	}
}





/**
 * Funcion callback a la que se accede cuando llega un dato del Gamepad.
 */
void funcionCallback(const std_msgs::Float32MultiArray::ConstPtr& array){

	// Pasamos el mensaje a un vector
    float botones[16];
    int i = 0;
    for(vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		botones[i] = *it;
		i++;
	}

	// Desgranamos el mensaje recibido.
    float jostick_left_x = botones[0];
    float jostick_left_y = botones[1];
    float jostick_rigth_x = botones[2];
    float jostick_rigth_y = botones[3];
    int L2 = botones[10];
    int R2 = botones[11];

    programacion_gatillos(L2,R2);

    ROS_INFO("Jostick Izquierdo Y = %f, Jostick Derecho Y = %f", jostick_left_y, jostick_rigth_y);
    ROS_INFO("Gatillo Izquierdo Y = %d, Gatillo Derecho Y = %d", L2, R2);
}


/**
 * Funcion Main de ROS.
 * Se encarga de inicializar ros y subscribirse al topic del gamepad.
 */
int main (int argc, char ** argv){

	// Registramos el nodo
	ros::init(argc,argv, nodo_name); // Nombre del nodo.
	ros::NodeHandle nodo; // Creamos el objeto nodo con el que interactuaremos.
	ROS_INFO("Nodo Gamepad creado y registrado."); // Mensaje de información

	// Ponemos en funcionamiento el modulo PWM
    Pwm.restart();
    Pwm.mode1_wake_up();
    Pwm.restart_values();
	// TODO: tenemos que poner algo de tiempo con ros
    //Pwm.set_pwm_tanto(2, 70);
    //Pwm.set_pwm_offset((1, 0, 100);
    //Pwm.set_pwm_offset((2, 0, 100);
    //Pwm.set_pwm_offset((3, 0, 100);


	// Subscribimos a gamepad
    ros::Subscriber subscriptor = nodo.subscribe(topic_name, 0, funcionCallback);

    ros::spin();
	return 0;
}