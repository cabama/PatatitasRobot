#include "ros/ros.h"
#include "patatitas/ultrasonidos.h"
#include "patatitas/encoders.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>
#include <sstream>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

using namespace std;

void split(const string& s, char c,
  vector<string>& v) {
  string::size_type i = 0;
  string::size_type j = s.find(c);

  while (j != string::npos) {
  v.push_back(s.substr(i, j-i));
  i = ++j;
  j = s.find(c, j);

  if (j == string::npos)
  v.push_back(s.substr(i, s.length()));
  }
}

int main(int argc, char** argv)
{
  int fd, n, i;
  char* buf;
  char* dataBuf;
  struct termios toptions;


  ros::init(argc, argv, "arduinoToRaspi_nodo");
  ros::NodeHandle nodo;
  ROS_INFO("Nodo que recoge datos de Arduino y publica los valores de los encoders y los ultrasonidos");

  ros::Publisher publicadorEncoders = nodo.advertise<patatitas::encoders>("encoders_topic", 0);
  ros::Publisher publicadorUltrasonidos = nodo.advertise<patatitas::ultrasonidos>("ultrasonidos_topic", 0);

  patatitas::encoders msjEncoders;
  patatitas::ultrasonidos msjUltrasonidos;


  /* open serial port */
  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
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

  //ros::Duration rate(1);

  while(true)
  {
    n = read(fd, dataBuf, 1);

    buf += dataBuf[0];
    /* insert terminating zero in the string */
    //buf[n] = 0;

    /*dos primeros valores encodes, y 5 siguientes ultrasonidos*/

    if(dataBuf == '\n')
    {
      printf("%i bytes read, buffer contains: %s\n", n, buf);

      std::string str = buf;

      vector<string> vectorString;
      split(str, ',', vectorString);

      //char* chars_array = strtok(buf, ",");
      int i=0;
      while(i<vectorString.size())
      {
        cout << atof(vectorString[i].c_str()) << ", ";
        if(i<=1)
          msjEncoders.encoders[i] = atof(vectorString[i].c_str());
        else
          msjUltrasonidos.ultrasonidos[i-2] = atof(vectorString[i].c_str());
        i++;
      }

      cout << endl;

      publicadorEncoders.publish(msjEncoders); 
      publicadorUltrasonidos.publish(msjUltrasonidos);
      
      buf = "";
    }

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
