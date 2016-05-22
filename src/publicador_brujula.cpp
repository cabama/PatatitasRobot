/**
 * Publicador_brujula.
 * Nodo de ROS que toma los datos de la brujula (hmc58883l) y los publica.
 * Para tomar los datos de la brujula se emplea la librería programada.
 * Desarrollado por PatatitasTeam
 */


// LIBRERIAS
#include <iostream>                 // Libreria Standar
#include "ros/ros.h"                // Libreria ROS
#include "std_msgs/Float64.h"  // Libreria Vector3 Mensaje de ROS para X Y Z.
#include "drivers/hmc5883l.cpp"       // Drivers de brujula

// NAMESPACES
using namespace std;

// VARIABLES
const string nodo_name = "nodo_brujula";    // Nombre del nodo.
const string topic_name = "brujula_bruto";  // Nombre del topic donde se va a publicar la brujula.
float tiempo_muestreo = 0.5;                // Ts | Tiempo de muestreo para la lectura de la brujula.
CompassMsg brujula_struct;                  // Struct desarrollado en la libreria de la brujula para recibir los tres datos.


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


/** 
 * Prototipo de la funcion calibración.
 * A esta funcion se le pasará el header bruto de la cabecera de la brujula
 */
double calibracion(double& matriz);


/**
 * Funcion principal donde se piden los datos a la libreria y se publican una vez tratados.
 */
int main(int argc, char **argv)
{

  // Iniciamos el nodo.
  ros::init(argc, argv, nodo_name);
  ros::NodeHandle nodo;
  ROS_INFO("nodo_brujula creado y registrado");
  // Cargamos el modulo de la brujula.
  HMC5883L brujula;
  CompassMsg mensaje_lib;
  int respuesta = brujula.conectamos_brujula();
  ROS_INFO("Modulo de la brujula cargado; (id-i2c = %d)", respuesta);
  // Definimos el publicador.
  ros::Publisher publicador = nodo.advertise<std_msgs::Float64>(topic_name, 0);
  ros::Duration seconds_sleep(tiempo_muestreo);

  // Bucle donde continuamente se va a realizar la lectura y publicacion de datos.
  // Este proceso se realizara cada float tiempo_muestreo
  while (ros::ok())
  {

    std_msgs::Float64 mensaje_ros;
    brujula_struct = brujula.get_data();
    printf("Magnetometro [x, y, z] = [%d, %d, %d] - Angulo = %f \n", brujula_struct.x, brujula_struct.y, brujula_struct.z, brujula_struct.angle);
    ROS_INFO("mensaje");
    publicador.publish(mensaje_ros);
    ros::spinOnce();
    seconds_sleep.sleep();
  }


  return 0;
}