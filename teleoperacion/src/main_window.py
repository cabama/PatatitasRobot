#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Este Script genera una ventana en la que se mostrara los sensores del robot patatitas
"""

import sys
from PyQt4 import QtCore, QtGui
from lib_windows.modulo_brujula import CompassWidget
from lib_windows.modulo_barras import Grafica
from lib_windows.modulo_speak import SpeakWidget
from lib_windows.modulo_data import LcdPatatitas
from lib_windows.conexiones_ros import conexiones_ros as ros
import random
import time
import threading

class Example(QtGui.QWidget):

    def __init__(self):
        super(Example, self).__init__()

        # Cargamos los modulos de los que depende esta clase
        self.compass = CompassWidget()
        self.grafica = Grafica()
        self.qtspeak = SpeakWidget(self)
        self.ros = ros(self) # Modulo que se encarga solo de las conexiones de ros 

        # Llamamos a la funcion de esta clase para cargar la interfaz grafica
        self.initUI()

    def initUI(self):

        # Creamos el titular
        self.title = QtGui.QLabel('Patatitas')
        self.title.setAlignment(QtCore.Qt.AlignCenter)
        self.title.setStyleSheet('color: green; font-size:40px; background-color: black')

        # Fila - con los bloques
        #self.lcd = QtGui.QLCDNumber(self)
        #self.lcd.setDigitCount(3)          # change the number of digits displayed
        #self.setWindowTitle('Encoder Izquierdo')
        #self.lcd.display("234")
        #self.lcd.setStyleSheet('color: green; font-size:40px; background-color: black')
        self.LCDencoderLeft = LcdPatatitas("Encoder Izquierdo","green")
        self.LCDencoderRigth = LcdPatatitas("Encoder Derecho","blue")
        self.LCDangulo = LcdPatatitas("Angulo","yellow")


        # Probando los LCDs
        layout_lcds = QtGui.QHBoxLayout()
        layout_lcds.addWidget(self.LCDencoderLeft)
        layout_lcds.addWidget(self.LCDencoderRigth)
        layout_lcds.addWidget(self.LCDangulo)


        #self.tituloUI() # Configuramos el titulo en una funcion
        layout_principal = QtGui.QHBoxLayout()
        layout_principal.addWidget(self.compass)
        layout_principal.addWidget(self.grafica)

        # Creamos el layout Contenedor de Windows y le añadimos los layout correspondientes
        layout_window = QtGui.QVBoxLayout()
        layout_window.addStretch(1)
        #layout_window.addWidget(self.title)
        layout_window.addLayout(layout_lcds)
        layout_window.addLayout(layout_principal)
        layout_window.addWidget(self.qtspeak)


        # Parametros de la ventana.
        self.setLayout(layout_window)
        self.setGeometry(300, 300, 1000, 400)
        self.setWindowTitle('Patatitas')
        self.show()

    def tituloUI(self):
        print("")


    def hilo(self):
        for i in range(1,100):
            num_grap = random.randint(0,359)
            num_bruj = random.randint(0,359)
            tims = i
            #self.compass.setAngle(num_grap)
            #self.grafica.add_sample((100,100,100,100,100))
            time.sleep(0.1)




    # ACTUADORES

    # Esta funcion se encarga de actualizar el modulo de la brujula desde el modulo de ros.
    # Actualmente tambien tiene valores aleatorios de prueba para otro modulo.

    def actualizar_brujula(self, value):
        # print ("dato del giroscopio: {}".format(value))
        self.compass.setAngle(value)
        self.LCDangulo.set_text(str(value))

        #valores = []
        #for valor in range(5):
        #    valores.append(random.randrange(100))
        # Probamos aunque aqui no sea la grafica
        #print ("valores {}".format(valores))
        #valores = (100, 100, 100, 100, 100) # valores de ejemplo
        #self.grafica.add_sample(valores)

    def actualizar_ultrasonidos(self, array_ultra):
        self.grafica.add_sample(array_ultra)


    # Este modulo se encarga de publicar a traves del modulo de ros, la frase que viene desde el modulo speak.
    def publicar_voz(self, frase):
        self.ros.publicador_voz(frase)



        


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon('lib_windows/resources/icon.png'))

    ex = Example()

    time.sleep(1)

    t = threading.Thread(target=ex.hilo)
    t.daemon = True
    t.start()

    sys.exit(app.exec_())

