import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import QtGui


from random import randint
import time
import threading

class SpeakWidget(QWidget):

    angleChanged = pyqtSignal(float)
        
    def __init__(self, main_class, parent=None):
        self.main_class = main_class
        super(SpeakWidget, self).__init__()
        self.initUI()

    def initUI(self):

        title = QtGui.QLabel('Patatitas Habla: ')
        self.speak_line = QtGui.QLineEdit()
        speakButton = QtGui.QPushButton("Habla!")
        speakButton.clicked.connect(self.clickedes)
        self.speak_line.editingFinished.connect(self.clickedes)

        layout_principal = QtGui.QHBoxLayout()
        layout_principal.addWidget(title)
        layout_principal.addWidget(self.speak_line)
        layout_principal.addWidget(speakButton)

        self.setLayout(layout_principal)
        self.show()

    def clickedes(self):
        frase = self.speak_line.text()
        if frase:
            self.main_class.publicar_voz(frase)
            self.speak_line.clear()

