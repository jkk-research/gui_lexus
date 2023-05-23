import sys
import os
import rclpy
from rclpy.node import Node

import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea

import json

class JSONReader:
    def __init__(self, filename):
        self.filename = filename
        self.data = []
        self.readData(self.filename)
    
    def readData(self, filename):
        with open(filename, "r") as f:
            self.data = json.load(f)

class PlotHandler(Node):
    def __init__(self):
        super().__init__('gui_buttons')
        scriptDir = os.path.dirname(os.path.realpath(__file__))
        try:
            buttonJSON = self.get_parameter('buttons').value
        except:
            self.declare_parameter('buttons', 'lexus_buttons.json')
            buttonJSON = self.get_parameter('buttons').value
        buttonReader = JSONReader(scriptDir + '/ReadFiles/' + buttonJSON)
        self.buttonData = buttonReader.data['buttons']
        print(self.buttonData)
        self.screenButtons = {}
        self.runningScreens = []
        pg.setConfigOptions(antialias=True)
        self.app = qtgqt.QtGui.QApplication([])

def main(args=None):
    rclpy.init(args=args)
    # print(scriptDir)
    ph = PlotHandler()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()