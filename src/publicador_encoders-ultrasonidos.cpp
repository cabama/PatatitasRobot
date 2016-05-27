#include "ros/ros.h"
#include "patatitas/ultrasonidos.h"
#include "patatitas/encoders.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>
#include <sstream>
#include <termios.h>

using namespace std;

int main(int argc, char** argv)
{
  int fd, n, i;
  char buf[64] = "temp text";
  struct termios toptions;


  ros::init(argc, argv, "Nodo publicador encoders y ultrasonidos");
  ros::NodeHandle nodo;
  ROS_INFO("Nodo que recoge datos de Arduino y publica los valores de los encoders y los ultrasonidos");

  ros::Publisher publicadorEncoders = nodo.advertise<patatitas::encoders>("encoders_topic", 0);
  ros::Publisher publicadorUltrasonidos = nodo.advertise<patatitas::ultrasonidos>("ultrasonidos_topic", 0);

  patatitas::encoders msjEncoders;
  patatitas::ultrasonidos msjUltrasonidos;


  /* open serial port */
  fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
  printf("fd opened as %i\n", fd);

  /* wait for the Arduino to reboot */
  usleep(3500000);

  /* get current serial port settings */
  tcgetattr(fd, &toptions);
  /* set 9600 baud both ways */
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* Canonical mode */
  toptions.c_lflag |= ICANON;
  /* commit the serial port settings */
  tcsetattr(fd, TCSANOW, &toptions);

  ros::Duration rate(1);

  while(ros::ok())
  {
    n = read(fd, buf, 64);
    /* insert terminating zero in the string */
    buf[n] = 0;

    /*dos primeros valores encodes, y 5 siguientes ultrasonidos*/
    printf("%i bytes read, buffer contains: %s\n", n, buf);

    char* chars_array = strtok(buf, ",");
    int i=0;
    while(i < 7)
    {
      if(i<=1)
        msjEncoders.encoders[i] = atof(chars_array[i]);
      else
        msjUltrasonidos.ultrasonidos[i-2] = atof(chars_array[i]);
      i++;
    }

    publicadorEncoders.publish(msjEncoders); 
    publicadorUltrasonidos.publish(msjUltrasonidos);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
