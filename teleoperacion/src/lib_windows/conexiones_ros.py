#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String


class conexiones_ros:



	def __init__(self, main_class):

		# Objeto de la clase principal de la que procede para poder hacer los callbacks
		self.main_class = main_class

		# Inicializamos el nodo
		rospy.init_node("interfaz_grafica", anonymous=True)

		# Nos subscribimos a los topics deseados
		#rospy.Subscriber("chatter", String, self.callback_brujula)
		#rospy.Subscriber("chatter", String, self.callback_acelerometro)
		rospy.Subscriber("rumbo_giroscopio", Float64, self.callback_giroscopio)

		# Publicador para mandar las frases que se escriben
		self.pub_voz = rospy.Publisher('espeaker_topic', String, queue_size=10)

		# Mantenemos el script de python despierto.
		#rospy.spin()

	def callback_brujula(self):
		return None

	def callback_acelerometro(self):
		return None

	def callback_ultrasonidos(self, data):
		valores = data.ultrasonidos
		self.main_class.actualizar_ultrasonidos(valores)

	# Retornamos el valor del giroscopio a la clase principal
	def callback_giroscopio(self, data):
		self.main_class.actualizar_brujula(data.data)

	def publicador_voz(self, frase):
		self.pub_voz.publish(str(frase))

