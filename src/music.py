#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to listen on a specific topic.
"""

# Import required Python code.
import rospy

# Import custom message data.
from std_msgs.msg import String

topic_speak = "espeaker_topic"


def callback(data):
    # Tomamos el mensaje
    frase = data.data
    # Imprimimos el dato recibido
    print("He escuchado: {}".format(frase))
    # Spliteamos para ver si es un comando
    spliteada = frase.split("play>")
    #Si se trata de un comando
    if res[0] is "" and res[1]:
        print("Voy a reproducir: {}".format(spliteada[1]))


def listener():
    '''
    Main function.
    '''
    # Create a subscriber with appropriate topic, custom message and name of
    # callback function.
    rospy.Subscriber(topic_speak, String, callback)
    # Wait for messages on topic, go to callback function when new messages
    # arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('musica')
    # Go to the main loop.
    listener()