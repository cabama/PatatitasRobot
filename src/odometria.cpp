#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <patatitas/encoders.h>

using namespace std;


// Callback del valor de los encoders
void encoders_callback(const patatitas::encoders::ConstPtr& encoders_msg){

}

// Callback del header del giroscopio
void giro_callback(const patatitas::encoders::ConstPtr& encoders_msg){
	
}

void gamepad_callback(const std_msgs::Float32MultiArray::ConstPtr& array){

	// Pasamos el mensaje a un vector
    float botones[16];
    int i = 0;
    for(vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		botones[i] = *it;
		i++;
	}

	// Desgranamos el mensaje recibido.
    float jostick_left_x = botones[0];
    float jostick_left_y = botones[1];
    float jostick_rigth_x = botones[2];
    float jostick_rigth_y = botones[3];
    int L1 = botones[8];
    int L2 = botones[10];
    int R2 = botones[11];
    int triangulo = botones[4];

    // TODO
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometria");
	ros::NodeHandle nh;

	ros::Subscriber encoders_sub = nh.subscribe("encoders", 10, encoders_callback);
	ros::Subscriber giro_sub = nh.subscribe("rumbo_giroscopio", 10, giro_callback);
	ros::Subscriber gamepad_sub = nh.subscribe("gamepad", 10, gamepad_callback);

	ros::spin();

	return 0;
}