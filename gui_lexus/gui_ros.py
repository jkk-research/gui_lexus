from __future__ import print_function

import sys
import os
import subprocess
import re
from functools import partial

import rclpy
from rclpy.node import Node

from .jsonreader import JSONReader


import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea

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

    def initButtons(self, widget):
        col, row = (0, 4)
        countRows = len(self.buttonData)//5
        for button in self.buttonData:
            buttonName, buttonLabel, buttonFunction = button["id"], button["label"], button["command"]
            buttonBgColor, buttonTextColor = "rgb(40, 44, 52)", "rgb(171, 178, 191)"
            if "bgColor" in button.keys():
                buttonBgColor = button["bgColor"]
            if "textColor" in button.keys():
                buttonTextColor = button["textColor"]
            
            self.screenButtons[buttonName] = qtgqt.QtGui.QPushButton(buttonLabel)
            
            widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
            
            buttonFunction = buttonFunction.replace("'","")
            buttonFunction = re.split(r'[,]\s*', buttonFunction)
            print(" ".join(buttonFunction))
            self.screenButtons[buttonName].clicked.connect(partial(self.buttonClicked, buttonFunction))
            self.screenButtons[buttonName].setStyleSheet("background-color: " + buttonBgColor + "; color: " + buttonTextColor)
            if col<4:
                col+=1
            else:
                col=0
                row+=1
        print(" ")

    def initializePlot(self):
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = "(16, 200, 166)"; greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        self.win.setWindowTitle("Screen handler")
        self.win.setFixedSize(800, 600)
        self.win.move(600, 200)
        self.win.setCentralWidget(area)
        
        dock1 = darea.Dock("", size = (1,1))  # give this dock minimum possible size
        area.addDock(dock1, "left")
        widg1 = pg.LayoutWidget()
        self.initButtons(widg1)
        self.updateBtn = qtgqt.QtGui.QPushButton("update screen list")
        self.wipeBtn = qtgqt.QtGui.QPushButton("wipe screens")
        
        WelcomeLabel = qtgqt.QtGui.QLabel("WELCOME TO LEXUS GUI")

        widg1.addWidget(self.wipeBtn, row=1, col=0)
        widg1.addWidget(self.updateBtn, row=1, col=4)
        widg1.addWidget(WelcomeLabel, row=0, col=2)
        widg1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock1.addWidget(widg1)
        self.state = None
        self.updateBtn.clicked.connect(self.update)
        self.wipeBtn.clicked.connect(self.wipeAllScreens)
        self.listwidget = qtgqt.QtGui.QListWidget()
        self.listwidget.setStyleSheet("""QListWidget{ color: rgb(171, 178, 191);}""")
        self.listwidget.clicked.connect(self.listclick)
        
        self.listwidget.itemDoubleClicked.connect(self.openscreen)
        self.listwidget
        
        dock1.addWidget(self.listwidget)

        self.update()
        self.timer = qtgqt.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000*10)
        self.win.show()
    
    def checkIfScreenIsRunning(self):
        p = subprocess.Popen(['screen', '-ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)        
        output, err = p.communicate()
        lines = output.splitlines()
        return {'localrun': [lines[0] == b'There is a screen on:' or lines[0] == b'There are screens on:', output]}

    def buttonClicked(self, command):
        executeCommand = []
        # ssh nvidia@192.168.1.5 screen -mdS mc2 bash -c "source ~/.bashrc&& mc"
        for i in range(0, len(command)-1):
            executeCommand.append(command[i])
        executeCommand.append('bash')
        executeCommand.append('-c')
        # executeCommand.append('"')
        executeCommand.append(command[len(command)-1])
        # executeCommand.append('"')
        print(" ".join(executeCommand))
        #print(sshCommand)
        #subprocess.check_call(sshCommand) 
        # TODO
        p = subprocess.Popen(executeCommand, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        #print(p.communicate()[0])
        self.update()


    def update(self):
        self.listwidget.clear()
        
        AllScreens = self.checkIfScreenIsRunning()

        if AllScreens['localrun'][0]:
            lines = AllScreens['localrun'][1].splitlines()
            print("Lines:", lines)
            for i in range(1, len(lines)-1):
                line = lines[i].decode('utf-8') 
                if line[0] == '\t':
                    self.listwidget.insertItem(0, line.split()[0].strip().split('.')[1])
                

    def openscreen(self):
        item = self.listwidget.currentItem()
        #print(item.text() + " >> double click")        
        toexec = ''.join(['screen -r ', str(item.text()), '; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', toexec])

    def listclick(self, qmodelindex):
        item = self.listwidget.currentItem()

    def wipeAllScreens(self):
        self.runningScreens = []

        AllScreens = self.checkIfScreenIsRunning()
        print(AllScreens)

        if AllScreens['localrun'][0]:
            lines = AllScreens['localrun'][1].splitlines()
            # print("Lines:", lines)
            for i in range(1, len(lines)-1):
                line = lines[i].decode('utf-8')
                PID = line.split()[0].strip().split('.')[1]
                p = subprocess.Popen(['screen', '-XS', PID, 'quit'])
        
        self.update()

def main(args=None):
    rclpy.init(args=args)
    # print(scriptDir)
    ph = PlotHandler()
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()