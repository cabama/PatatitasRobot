import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import QtGui


from random import randint
import time
import threading

class LcdPatatitas(QWidget):

        
    def __init__(self, titulo, color, parent=None):
        super(LcdPatatitas, self).__init__()
        self.titulo = titulo
        self.color = color
        # Iniciamos la interfaz
        self.initUI()


    def initUI(self):

        self.title = QtGui.QLabel(self.titulo)
        self.title.setAlignment(Qt.AlignCenter)

        self.databox = QtGui.QLabel('00')
        self.databox.setAlignment(Qt.AlignCenter)
        self.databox.setStyleSheet('color: {0}; font-size:30px; background-color: black'.format(self.color))

        layout_principal = QtGui.QVBoxLayout()
        layout_principal.addWidget(self.title)
        layout_principal.addWidget(self.databox)

        self.setLayout(layout_principal)
        self.show()

    def set_text(self, text):
        self.databox.setText(text)


