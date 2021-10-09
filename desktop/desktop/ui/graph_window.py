from PyQt5 import QtWidgets
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os

class GraphWindow(QtWidgets.QMainWindow):

    arr_x = list()
    arr_y = list()

    def __init__(self, *args, **kwargs):
        super(GraphWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        # plot data: x, y values
        # self.graphWidget.plot(hour, temperature)
    
    def plot(self, x_pos, y_pos):
        self.graphWidget.plot(x_pos, y_pos)