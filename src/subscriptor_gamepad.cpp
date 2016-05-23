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
 * [0] -> Jostick izquierdo, eje x.
 * [1] -> Jostick izquierdo, eje y.
 * [2] -> Jostick derecho, eje x.
 * [3] -> Jostick derecho, eje y.
 * [4] -> Triangulo.
 * [5] -> Circulo.
 * [6] -> Equis.
 * [7] -> Cuadrado.
 * [8] -> L1.
 * [9] -> R1.
 * [10] -> L2.
 * [11] -> R2.
 * [12] -> Select.
 * [13] -> Start.
 * [14] -> L3.
 * [15] -> R3.
 */

// LIBRERIAS.
#include "ros/ros.h"
#include "beginner_tutorials/mensajeTest.h"
#include "std_msgs/Float32MultiArray.h"


// VARIABLES GLOBALES.
const string nodo_name = "gamepad_subscriptor";
const string topic_name = "gamepad";



/**
 * Funcion callback a la que se accede cuando llega un dato del Gamepad.
 */
void funcionCallback(const  std_msgs::Float32MultiArray::ConstPtr& msg){

	// Desgranamos el mensaje recibido.
    Float botones[] = msg.data;
    float jostick_left_x = botones[0];
    float jostick_left_y = botones[1];
    float jostick_rigth_x = botones[1];
    float jostick_rigth_y = botones[1];

    // Movemos motor izquierdo.

    // Movemos motor derecho.

    // Giro sobre si mismo izquierdo.

    // Giro sobre si mismo derecho


    ROS_INFO("Jostick Izquierdo Y = %d, Jostick Derecho Y = %d", jostick_left_y, jostick_rigth_y);
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

	// Subscribimos a gamepad
    ros::Subscriber subscriptor = nodo.subscribe(topic_name, 0, funcionCallback);

    ros::spin();
	return 0;
}