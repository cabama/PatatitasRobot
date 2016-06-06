# Gamepad
import pygame
import time
from pygame.locals import *
# ROS
import rospy
import roslib
from std_msgs.msg import Float32MultiArray

# Importamos el mensaje que vamos a utilizar
# roslib.load_manifest('aristarko')
# from aristarko.msg import sensor_hdc1008 as msg_hdc1008


class GamePad:

    publicador = None

    def __init__(self):
        pygame.init()
        self.j = pygame.joystick.Joystick(0)
        self.j.init()
        print 'Initialized Joystick : %s' % self.j.get_name()
        self.init_node()


    def init_node(self):
        self.publicador = rospy.Publisher("gamepad", Float32MultiArray, queue_size=0)
        rospy.init_node("GamePadPC", anonymous=True)
        self.rate = rospy.Rate(10) # 10hz



    def get(self):
        mando = []
        joystick = []
        buttons = []
        pygame.event.get()

        #Read input from the two joysticks       
        for i in [0,1,3,4]:
            joystick.append(self.j.get_axis(i))
            mando.append(self.j.get_axis(i))

        #Read input from buttons
        for i in range(0, self.j.get_numbuttons()):
            buttons.append(self.j.get_button(i))
            mando.append(self.j.get_button(i))
            
        return joystick, buttons, mando


    def publicar_mensaje(self, array):
        mensaje = Float32MultiArray()
        mensaje.data = array
        rospy.loginfo(mensaje)
        self.publicador.publish(mensaje)


    def test(self):
        while True:
            jj,bb, mando = self.get()
            print jj
            print bb
            time.sleep(0.1)


if __name__ == '__main__':
    pad = GamePad()
    #_,_,mando = pad.test()

    while not rospy.is_shutdown():
        jj,bb, mando = pad.get()
        print jj
        print mando
        pad.publicar_mensaje(mando)
        pad.rate.sleep()


