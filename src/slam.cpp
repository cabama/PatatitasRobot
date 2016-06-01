#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
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

// Tamaño del robot (metros)
#define ROBOT_WIDTH 0.3
#define ROBOT_LENGTH 0.7

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
Mat mapa(MAP_WIDTH/CELL_SIZE, MAP_HEIGHT/CELL_SIZE, CV_8UC1, Scalar(UNKNOWN));

// Posicion del robot
float x_pos = 0;
float y_pos = 0;
float header = 0;

// Parametros de la posicion de los ultrasonidos
// Ángulos: -90, -45, 0, 45, 90. Senos y cosenos de cada uno precalculados
float ultra_senos[5] = {1, 0.70710678, 0, -0.70710678, -1};
float ultra_cosenos[5] = {0, 0.70710678, 1, 0.70710678, 0};
// Distancias de cada sensor al centro del robot en x,y precalculadas (en metros)
float ultra_x[5] = {0.07, 0.06, 0.0, -0.06, -0.07};
float ultra_y[5] = {0.0, 0.12, 0.14, 0.12, 0.0};


void publica_mapa()
{
	// Clona el mapa y pinta encima el robot
	Mat draw_map = mapa.clone();
	cvtColor(draw_map, draw_map, CV_GRAY2BGR);

	// Dibuja un rectabgulo en la posicion del robot
	RotatedRect stamp = RotatedRect(
	    Point2f((MAP_WIDTH/2 +  x_pos) / CELL_SIZE, (MAP_HEIGHT/2 - y_pos) / CELL_SIZE),
	    Size2f(ROBOT_WIDTH / CELL_SIZE, ROBOT_LENGTH / CELL_SIZE),
	    header
	);
	Point2f vertices[4];
	stamp.points(vertices);
	const Point polyPoints[4] = { vertices[0], vertices[1], vertices[2], vertices[3] };

	fillConvexPoly(draw_map, polyPoints, 4, Scalar(255,0,0));

	// Resize para que sea "visible"
	resize(draw_map, draw_map, Size(600,600));

	// Muestra el mapa
	imshow("MAPA", draw_map);
	// Publica la imagen
	cv_bridge::CvImage img;
    img.encoding = "bgr8";
    img.header.stamp = ros::Time::now();
    img.image = draw_map;
    map_pub.publish(img.toImageMsg());
}



void odometria_callback(const geometry_msgs::Pose2D::ConstPtr& odom_msg)
{
	x_pos = odom_msg->x;
	y_pos = odom_msg->y;
	header = odom_msg->theta;
	publica_mapa();
}


// Add one obstacle in meters relative to the vehicle and transform into map coordinates.
void add_obstacle(float obstacle_pos_x, float obstacle_pos_y){
	Point obstacle;
	obstacle.x = (x_pos + obstacle_pos_x + MAP_WIDTH/2) / CELL_SIZE;
	obstacle.y = - (y_pos + obstacle_pos_y - MAP_HEIGHT/2) / CELL_SIZE;

	// Aplica una rotacion a los puntos de los obstaculos con respecto al centro del robot en funcion del header actual
	Mat rot_mat = getRotationMatrix2D( Point((x_pos + MAP_WIDTH/2) / CELL_SIZE, (MAP_HEIGHT/2 - y_pos) / CELL_SIZE), header, 1);
	Point rotated;
	rotated.x = rot_mat.at<double>(0,0) * obstacle.x + rot_mat.at<double>(0,1) * obstacle.y + rot_mat.at<double>(0,2);
    rotated.y = rot_mat.at<double>(1,0) * obstacle.x + rot_mat.at<double>(1,1) * obstacle.y + rot_mat.at<double>(1,2);

	circle(mapa, rotated, 2, Scalar(OBSTACLE), CV_FILLED);
}

// Add a free line from the robot position to the obstacle in map coordinates
void add_free_line(float obstacle_pos_x, float obstacle_pos_y){
	Point obstacle;
	obstacle.x = (x_pos + obstacle_pos_x + MAP_WIDTH/2) / CELL_SIZE;
	obstacle.y = - (y_pos + obstacle_pos_y - MAP_HEIGHT/2) / CELL_SIZE;

	// Aplica una rotacion a los puntos de los obstaculos con respecto al centro del robot en funcion del header actual
	Mat rot_mat = getRotationMatrix2D( Point((x_pos + MAP_WIDTH/2) / CELL_SIZE, (MAP_HEIGHT/2 - y_pos) / CELL_SIZE), header, 1);
	Point rotated;
	rotated.x = rot_mat.at<double>(0,0) * obstacle.x + rot_mat.at<double>(0,1) * obstacle.y + rot_mat.at<double>(0,2);
    rotated.y = rot_mat.at<double>(1,0) * obstacle.x + rot_mat.at<double>(1,1) * obstacle.y + rot_mat.at<double>(1,2);

    Point start_point;
	start_point.x = (x_pos + MAP_WIDTH/2) / CELL_SIZE;
	start_point.y = - (y_pos - MAP_HEIGHT/2) / CELL_SIZE;

    line(mapa, start_point, rotated, Scalar(FREE), 2);
}

void ultrasonidos_callback(const patatitas::ultrasonidos::ConstPtr& ultrasonidos_msg)
{
	for (int i = 0; i < 5; ++i)
	{
		// Distancia del obstaculo al sensor (convertidas de cm a m)
		float obs_distance = ultrasonidos_msg->ultrasonidos[i] / 100;
		// Calcula la posicion x,y real del obstaculo respecto del robot
		float obs_distance_x = obs_distance * ultra_senos[i] + ultra_x[i];
		float obs_distance_y = obs_distance * ultra_cosenos[i] + ultra_y[i];

		// Pinta la zona libre en el mapa
		add_free_line(obs_distance_x, obs_distance_y);
		// Pinta el obstáculo en el mapa
		add_obstacle(obs_distance_x, obs_distance_y);
	}
	publica_mapa();
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