#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

const string nodo = "nodochatter";
const string topic = "topicChatter";

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, nodo);

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(topic, 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  { 
    // Creams un mensaje
    std_msgs::String msg;
    // Creamos el string
    std::stringstream ss;
    ss << "hello world " << count;
    // Metemos el string en el mensaje
    msg.data = ss.str();
    // Publicamos el log y el mensaje
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    // Esperamos y contamos un nuevo mensaje
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}