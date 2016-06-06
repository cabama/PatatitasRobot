import sys
from PyQt4.QtGui import *
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt
import numpy as np
import collections


class Grafica(QWidget):

    data = [0,0,0,0,0,0,0,0,0,0]
    time_grap = [0,0,0,0,0,0,0,0,0,0]

    def __init__(self, parent=None):
        super(Grafica, self).__init__()

        # FIGUREANDO
        self.figure = plt.figure()
        self.line, = plt.plot(self.data)
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


    def add_sample(self,x,y):
        self.data.append(y)
        self.time_grap.append(x)
        self.data.pop(0)
        self.time_grap.pop(0)

        self.ax.hold(False)
        self.ax.set_ylim([0, 360])
        self.ax.set_xlim([0, 360])
        self.ax.plot(self.time_grap, self.data)
        self.canvas.draw()  # update the plot
