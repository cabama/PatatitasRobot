#include "ros/ros.h"
#include "Patatitas_Robot/brujula_msgs.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{


    ros::init(argc, argv, "nodo_brujula");
    ros::NodeHandle nodo;
    ROS_INFO("nodo_emisor creado y registrado");
    ros::Publisher publicador = nodo.advertise<Patatitas_Robot::{brujula_msgs:mensajeTest}>("brujula_bruto", 0);
    ros::Duration seconds_sleep(1);


  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    beginner_tutorials::brujula_msgs mensaje;
    mensaje.numero = count;
    publicador.publish(mensaje);

    ros::spinOnce();

    seconds_sleep.sleep();
    ++count;
  }


  return 0;
}