#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "drivers/GY-521.cpp"
#include <sstream>

//ubuntu@ubuntu:~/catkin_ws$ g++ brujula.cpp -o example_brujula -lwiringPi -lpthread


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    // Inicializamos y registramos el nodo
    ros::init(argc, argv, "nodo_gy521");
    ros::NodeHandle nodo;
    ROS_INFO("nodo_gy521: publicador giroscopio y acelerometro, creado y registrado");

    // Registramos los publicadores
    ros::Publisher publicador_acel = nodo.advertise<geometry_msgs::Vector3>("acelerometro_bruto", 0);
    ros::Publisher publicador_gyros = nodo.advertise<geometry_msgs::Vector3>("giroscopio_bruto", 0);
    ros::Duration seconds_sleep(0.2);
    // Conectamos con el acelerometro
    MPU6050 acelerometro;
    acelerometro.conectamos_acelerometro();



  while (ros::ok())
  {

    // Declaramos las dos estructuras y la clase acelerometro para su control
    Espacio aceleracion, gyroscocion;
    geometry_msgs::Vector3 aceleraciones, giroscopios;

    // Obtenemos datos del acelerometro
    aceleracion = acelerometro.get_aceleraciones();
    gyroscocion = acelerometro.get_giroscopio();

    // Convertimos el mensaje tipo Espacio a ROS Geometry Vector3
    aceleraciones.x = aceleracion.x;
    aceleraciones.y = aceleracion.y;
    aceleraciones.z = aceleracion.z;
    giroscopios.x = gyroscocion.x;
    giroscopios.y = gyroscocion.y;
    giroscopios.z = gyroscocion.z;

    //Publicamos los datos    
    publicador_acel.publish(aceleraciones);
    publicador_gyros.publish(giroscopios);

    ros::spinOnce();
    seconds_sleep.sleep();
  }


  return 0;
}     