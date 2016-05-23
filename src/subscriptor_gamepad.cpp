/**
 * Subscriptor_gamepad.cpp
 * Este nodo se encarga de subscribirse al topic del gamepad que se publica desde el otro PC.
 * A partir de los datos recibidos 
 */


// LIBRERIAS.
#include "ros/ros.h"
#include "beginner_tutorials/mensajeTest.h"
#include "std_msgs/Float32MultiArray.h"


// VARIABLES GLOBALES.
const string nodo_name = "gamepad_subscriptor";
const string topic_name = "gamepad";


/**
 * 
 * Estructura del mensaje GamePad.
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


/**
 * Funcion callback a la que se accede cuando llega un dato del Gamepad.
 */
void funcionCallback(const  std_msgs::Float32MultiArray::ConstPtr& msg){
    Float botones[] = msg.data;
    ROS_INFO("");
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