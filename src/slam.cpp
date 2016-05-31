#include <iostream>
#include <opencv2/opencv.hpp>
#include <math>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <patatitas/ultrasonidos.h>

using namespace std;
using namespace cv;

// Medidas del mapa (ancho y alto en metros)
#define MAP_WIDTH 20
#define MAP_HEIGHT 20
// Resolucion del mapa (metros / celda)
#define CELL_SIZE 0.1


ros::Publisher map_pub;

// Mapa
/* Mascara del mapa
*	100	->	UNKNOWN
*	0	->	OBSTACLE
*	255	->	FREE
*/
#define UNKNOWN 100
#define OBSTACLE 0
#define FREE 255
Mat map(MAP_WIDTH/CELL_SIZE, MAP_HEIGHT/CELL_SIZE, CV_8UC1, Scalar(UNKNOWN));

// Posicion del robot
float x_pos = 0;
float y_pos = 0;
float header = 0;

// Vector con las posiciones de los obstáculos (x,y)
vector<Point2f> obstacles;

void odometria_callback(const geometry_msgs::Pose2D::ConstPtr& odom_msg)
{
	x_pos = odom_msg->x;
	y_pos = odom_msg->y;
	header = odom_msg->theta;

}


// Add one obstacle in meters relative to the vehicle and transform into map coordinates.
void add_obstacle(float obstacle_pos_x, float obstacle_pos_y){
	Point obstacle;
	obstacle.x = (x_pos + obstacle_pos_x + MAP_WIDTH/2) / CELL_SIZE;
	obstacle.y = - (y_pos + obstacle_pos_y - MAP_HEIGHT/2) / CELL_SIZE;

	circle(map, obstacle, 3, Scalar(OBSTACLE), CV_FILLED);
}

// Add a free line from the robot position to the obstacle in map coordinates
void add_free_line(float obstacle_pos_x, float obstacle_pos_y){
	Point obstacle;
	obstacle.x = (x_pos + obstacle_pos_x + MAP_WIDTH/2) / CELL_SIZE;
	obstacle.y = - (y_pos + obstacle_pos_y - MAP_HEIGHT/2) / CELL_SIZE;

    Point start_point;
	start_point.x = (x_pos + MAP_WIDTH/2) / CELL_SIZE;
	start_point.y = - (y_pos - MAP_HEIGHT/2) / CELL_SIZE;

    line(map, start_point, obstacle, Scalar(FREE), 2);
}

void ultrasonidos_callback(const patatitas::ultrasonidos::ConstPtr& ultrasonidos_msg)
{
	// Ángulos: -90, -45, 0, 45, 90
	for (int i = 0; i < 5; ++i)
	{
		// Distancia del obstaculo al sensor en:
		ultrasonidos_msg->ultrasonidos[i];
		// TODO: Calcular posicion x,y real del obstaculo y pintar en el mapa
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "slam");
	ros::NodeHandle nh;

	map_pub = nh.advertise<sensor_msgs::Image>("map", 10);

	ros::Subscriber odometria_sub = nh.subscribe("odometria", 10, odometria_callback);
	ros::Subscriber ultrasonidos_sub = nh.subscribe("ultrasonidos_topic", 10, ultrasonidos_callback);

	ros::spin();

	return 0;
}