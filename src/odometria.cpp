#include <iostream>
#include <math>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <patatitas/encoders.h>

using namespace std;

ros::Publisher odometria_pub;


// Ticks del encoder por cada vuelta
#define TICKS_PER_REVOLUTION 40
// Diámetro de la rueda
#define WHEEL_DIAMETER 0.07


// Acciones de control del gamepad
enum control_action
{
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	STOP
};
control_action current_action;


// Valores de los encoders
int last_left_encoder_tics = 0;
int last_right_encoder_tics = 0;
int current_left_encoder_tics = 0;
int current_right_encoder_tics = 0;

// Posicion
float x_pos = 0;
float y_pos = 0;
// Header
float header = 0;


void publica_pose()
{
	// Publicar la pose (x,y,header)
	geometry_msgs::Pose2D pose;
	pose.x = x_pos;
	pose.y = y_pos;
	pose.theta = header;
	odometria_pub.publish(pose);
}


// Callback del valor de los encoders
void encoders_callback(const patatitas::encoders::ConstPtr& encoders_msg)
{
	current_left_encoder_tics = encoders_msg->encoders[0];
	current_right_encoder_tics = encoders_msg->encoders[1];

	// Diferencias de los encoders (avance en ticks de cada rueda)
	int left_encoder_diff = last_left_encoder_tics - current_left_encoder_tics;
	int right_encoder_diff = last_right_encoder_tics - current_right_encoder_tics;

	// Comprobar el overflow de la variable.
	if (left_encoder_diff < 0 || right_encoder_diff < 0)
	{
		last_left_encoder_tics = current_left_encoder_tics;
		last_right_encoder_tics = current_right_encoder_tics;
		return;
	}

	// Distancias reales (avance en metros de cada rueda)
	float left_distance = M_PI * WHEEL_DIAMETER * left_encoder_diff / TICKS_PER_REVOLUTION;
	float right_distance = M_PI * WHEEL_DIAMETER * right_encoder_diff / TICKS_PER_REVOLUTION;
	float avance = (left_distance + right_distance) / 2;


	// En función de la accion ejecutada se actualiza la posicion
	switch(current_action)
	{
		case  FORWARD:
			// TODO: Calcular radio de giro
			float avance_x = - avance * sin(header);
			float avance_y = avance * cos(header);
			x_pos += avance_x;
			y_pos += avance_y;
			break;
		case  BACKWARD:
			// TODO
			float avance_x = avance * sin(header);
			float avance_y = - avance * cos(header);
			x_pos += avance_x;
			y_pos += avance_y;
			break;
		case  LEFT:
			// El robot no deberia avanzar. Gira sobre si mismo.
			// TODO: Comprobar que esto se cumple y corregir si es necesario
			break;
		case  RIGHT:
			// El robot no deberia avanzar. Gira sobre si mismo.
			// TODO: Comprobar que esto se cumple y corregir si es necesario
			break;
		case  STOP:
			// El robot no deberia avanzar.
			break;
	}

	publica_pose();

	// Actualiza los valores de los encoders
	last_left_encoder_tics = current_left_encoder_tics;
	last_right_encoder_tics = current_right_encoder_tics;

}

// Callback del header del giroscopio
void giro_callback(const patatitas::encoders::ConstPtr& gyro_msg)
{
	// Valor en bruto del giro. Sin filtrar ni acotar entre 0-360.
	float gyro_raw = gyro_msg->data;

	// Acotamos el valor entre 0-360
	header = gyro_raw % 360;
	publica_pose();
}

void gamepad_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	// Pasamos el mensaje a un vector
    float botones[16];
    int i = 0;
    for(vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		botones[i] = *it;
		i++;
	}

	// Desgranamos el mensaje recibido.

    int L1 = botones[8];
    int L2 = botones[10];
    int R2 = botones[11];

    // Actualizar la accion actual
	if (L1 == 1)
	{
		current_action = BACKWARD;
	}
	else
	{
	    if (L2 == 1 && R2 == 0)
	    {
	    	current_action = RIGHT;
	    }
	    else if (L2 == 0 && R2 == 1)
	    {
	    	current_action = LEFT;
	    }
	    else if (L2 == 1 && R2 == 1)
	    {
	    	current_action = FORWARD;
	    }
	    else if (L2 == 0 && R2 == 0)
	    {
	    	current_action = STOP;
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometria");
	ros::NodeHandle nh;

	odometria_pub = nh.advertise<geometry_msgs::Pose2D>("odometria", 10);

	ros::Subscriber encoders_sub = nh.subscribe("encoders", 10, encoders_callback);
	ros::Subscriber giro_sub = nh.subscribe("rumbo_giroscopio", 10, giro_callback);
	ros::Subscriber gamepad_sub = nh.subscribe("gamepad", 10, gamepad_callback);

	ros::spin();

	return 0;
}