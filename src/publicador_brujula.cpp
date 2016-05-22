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

// Parametros de calibracion de la brujula
double bias[3] = {-112.017978, -39.199350, -97.169059};
// Filas de la matriz
double A_0[3] = {0.089225, -0.001845, 0.006807};
double A_1[3] = {-0.001845, 0.090054, 0.002206};
double A_2[3] = {0.006807, 0.002206, 0.092495};



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


/** 
 * Prototipo de la funcion calibración.
 * Recibe los valores x, y, z brutos de la brujula y los corrige
 */
void calibracion(CompassMsg &mag){
  mag.x -= bias[0];
  mag.y -= bias[1];
  mag.z -= bias[2];
  double new_x = A_0[0]*mag.x + A_0[1]*mag.y + A_0[2]*mag.z;
  double new_y = A_1[0]*mag.x + A_1[1]*mag.y + A_1[2]*mag.z;
  double new_z = A_2[0]*mag.x + A_2[1]*mag.y + A_2[2]*mag.z;
  mag.x = new_x;
  mag.y = new_y;
  mag.z = new_z;
}


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
  CompassMsg brujula_struct;
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
    calibracion(brujula_struct);
    printf("Magnetometro [x, y, z] = [%d, %d, %d] - Angulo = %f \n", brujula_struct.x, brujula_struct.y, brujula_struct.z, brujula_struct.angle);
    ROS_INFO("mensaje");
    publicador.publish(mensaje_ros);
    ros::spinOnce();
    seconds_sleep.sleep();
  }


  return 0;
}