import sys
from PyQt4.QtGui import *
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt
import numpy as np
import collections


class Grafica(QWidget):

    valores = (20, 35, 30, 35, 27) # valores de ejemplo


    def __init__(self, parent=None):
        super(Grafica, self).__init__()

        # FIGUREANDO
        self.ordenadas = np.arange(5)
        self.width = 1       # the width of the bars
        self.figure, self.ax = plt.subplots()
        #self.figure = plt.figure()
        self.line = self.ax.bar(self.ordenadas, self.valores, self.width, color='g')
        #self.line, = plt.plot(self.data)
        plt.ion() # animate

        N = 10
        self.xs = collections.deque(maxlen=N)
        self.ys = collections.deque(maxlen=N)
        self.xs.append(0)
        self.ys.append(0)

        self.ax = self.figure.add_subplot(111)
        self.ax.hold(False)
        self.ax.set_ylim([0, 360])

        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.hide()
        self.canvas.show()

        # set the layout
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.canvas)
        self.setLayout(self.layout)


    def add_sample(self, valores):
        self.valores = valores
        self.line = self.ax.bar(self.ordenadas, self.valores, self.width, color='g')
        self.canvas.draw()  # update the plot
