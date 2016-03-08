#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  // Iniciamos el nodo y le pasamos los argumentos
  ros::init(argc, argv, "talker");
  // NodeHandle es el principal acceso para las comunicaciones con el sistema ROS.
  ros::NodeHandle n;
  // Creamo el topic
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Configuramos la frecuencia de publicacion
  ros::Rate loop_rate(10);

  /**
   * Bucle en el que publicaremos a la frecuencia indicada un mensaje por el topic
   */
  int count = 0;
  while (ros::ok())
  {
    
    // EMPAQUETAMOS EL MENSAJE
      // Primero creamos el mensaje
      std_msgs::String msg;
      // Luego lo empaquetamos
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

    // Informacion que se muestra en consola hacerca del mensaje {comentar si no se debuggea}
    ROS_INFO("%s", msg.data.c_str());

    // PUBLICAMOS EL MENSAJE
    chatter_pub.publish(msg);

    // Esperamos el tiempo para que se cumpla le frecuencia indicada
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }


  return 0;
}