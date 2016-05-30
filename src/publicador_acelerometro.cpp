#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "drivers/GY-521.cpp"
#include <sstream>
#include "std_msgs/Float64.h"

//ubuntu@ubuntu:~/catkin_ws$ g++ brujula.cpp -o example_brujula -lwiringPi -lpthread


// Parametros de calibracion del giroscopio
double bias[3] = {0, 0, 0};
const uint sgyro = 131;// Sensitivity scale factor (LSB/(º/s))


/** 
 * Cálculo del offset
 * Recibe los valores x, y, z brutos de la brujula y calcula el offset
 */
void calcular_ofset(Espacio &gyroscocion){

  bias[0] = 0.9*bias[0] + 0.1*gyroscocion.x;
  bias[1] = 0.9*bias[1] + 0.1*gyroscocion.y;
  bias[2] = 0.9*bias[2] + 0.1*gyroscocion.z;
}

/** 
 * Prototipo de la funcion calibración.
 * Recibe los valores x, y, z brutos de la brujula y los corrige
 */
void calibracion(Espacio &gyroscocion){

  gyroscocion.x = (gyroscocion.x - bias[0])/sgyro;
  gyroscocion.y = (gyroscocion.y - bias[1])/sgyro;
  gyroscocion.z = (gyroscocion.z - bias[2])/sgyro;
  // TODO modificar el struc.angle para actualizar el header. 
}

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
    ros::Publisher publicador_headerGyro = nodo.advertise<std_msgs::Float64>("rumbo_giroscopio", 0);
    ros::Duration seconds_sleep(0.2);
    // Conectamos con el acelerometro
    MPU6050 acelerometro;
    acelerometro.conectamos_acelerometro();
    Espacio aceleracion, gyroscocion;

    // Declaramos el yaw a calcular mediante el giroscopio
    std_msgs::Float64 gyro_header;
    // Inicializamos el rumbo del acelerómetro (º/s)
    gyro_header = 0;

    // Calculamos el offset con las primeras 500 primeras muestras del giroscopio
    for (int i=0; i < 500; ++i){
      gyroscocion = acelerometro.get_giroscopio();

      // Calibrar giroscopio
      calcular_ofset(gyroscocion);      
    }


  while (ros::ok())
  {

    // Declaramos las dos estructuras y la clase acelerometro para su control
    geometry_msgs::Vector3 aceleraciones, giroscopios;

    // Obtenemos datos del acelerometro
    aceleracion = acelerometro.get_aceleraciones();
    gyroscocion = acelerometro.get_giroscopio();

    // Calibramos giroscopio
    calibracion(gyroscocion);

    // Obtenemos rumbo mediante el giroscopio
    gyro_header.data += giroscocion.z;

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
    publicador_headerGyro.publish(gyro_header);

    ros::spinOnce();
    seconds_sleep.sleep();
  }


  return 0;
} 