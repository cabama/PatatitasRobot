#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

const string nodo_name = "espeaker";
const string topic_name = "espeaker_topic";



void funcionCallback(const  std_msgs::String::ConstPtr& msg)
{
	string command = "espeak -v es \"" + msg->data +"\" 2>/dev/null";
    system (command.c_str());
}




int main (int argc, char ** argv)
{

	ros::init(argc,argv, nodo_name); // Nombre del nodo.
	ros::NodeHandle nodo; // Creamos el objeto nodo con el que interactuaremos.
	ROS_INFO("nodo_receptor creado y registrado."); // Mensaje de información
    ros::Subscriber subscriptor = nodo.subscribe(topic_name ,0, funcionCallback);

    ros::spin();
	return 0;

}