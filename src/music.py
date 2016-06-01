#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

# Import required Python code.
import rospy
import os, signal, subprocess

# Import custom message data.
from std_msgs.msg import String


class Reproductor:

    topic_speak = "espeaker_topic"
    reproductor = None

    def __init__(self):
        rospy.init_node('musica')
        rospy.Subscriber(self.topic_speak, String, self.callback)
        rospy.spin()

    def callback(self, data):
        # Tomamos el mensaje
        frase = data.data

        # Imprimimos el dato recibido
        print("He escuchado: {}".format(frase))
        
        # Spliteamos para ver si es un comando
        splitPlay = frase.split("play>")
        splitStop = frase.split("stop>")

        #Si se trata de un comando
        if splitPlay[0] is "" and splitPlay[1]:
            musica = "/home/butakus/catkin/src/PatatitasRobot/src/resources/music/"+splitPlay[1]
            print("Voy a reproducir: {}".format(musica))
            self.reproductor = subprocess.Popen(args=["omxplayer", musica])
        elif splitStop[0] is "":
            print("Voy a parar la musica, voy a parar el PID {}".format(self.reproductor.pid))
            self.reproductor.terminate()
            subprocess.Popen(args=["pkill", "-f", "omxplayer"])




# Main function.
if __name__ == '__main__':
    
    # Go to the main loop.
    Reproductor()