import sys
import os
import random
import matplotlib
import matplotlib.pyplot as plt
import time
import math
import csv
import ast
#rom scipy import spatial

# Make sure that we are using QT5
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
QSizePolicy, QVBoxLayout, QWidget,QLineEdit, QInputDialog)

from numpy import arange
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

from enum import Enum
from laserGlobals import laserGlobals

progname = os.path.basename(sys.argv[0])

#Arxikopoihsh global timwn
#Arxikopoihsh metritwn
cnt = 0
timer = None
#Arxikopoihsh grafikwn parastasewn
ax = None
fig = None
fw = None
scan_widget = None
ok = 'No'
#Arxikopoihsh annotation
objx = []
objy = []
annotating = False
firstclick = False
secondclick = False
thirdclick = False
colours = ['#FF69B4','#FFFF00','#CD853F','#000000','#8A2BE2','#00BFFF','#ADFF2F','#8B0000']
colour_index = 0
c1 = []
c2 = []
colorName = []
txt = None
annot = []
classes = None
selections = []
le = None
objs = None
reinitialize = False

class Window(FigureCanvas):

    def __init__(self, parent=None, width=10, height=3, dpi=100):

        global fw,fig,ax,bag_file,data

        fw = self

        fig = Figure(figsize=(width, height), dpi=dpi)

        self.axes = fig.add_subplot(111)

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def icon(self):
        pass


class LS(Window):

    def ptime(self):
        laserGlobals.timer = QtCore.QTimer(None)

        laserGlobals.timer.timeout.connect(self.icon)
        laserGlobals.timer.start(100)
    

    def icon(self):

        global cnt,annot,ok

        if(cnt<len(annot)):
            ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
            cnt += 1
        if (cnt == len(annot)):
            cnt=0
            laserGlobals.timer.stop()
            ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def drawLaserScan(self):

        global ax,annot,cnt,samex,samey,listofpointsx,listofpointsy,fw,ok,c1,c2,colorName,firstclick,secondclick,colourID,colorName

        if (ok == 'Yes'):
            self.axes.clear()
            self.axes.axis('equal')
            self.axes.plot(annot[cnt].samex,annot[cnt].samey,'bo')
            if not annot[cnt].listofpointsx == []:
                for j in range(len(annot[cnt].colourID)):
                    self.axes.plot(annot[cnt].listofpointsx[j],annot[cnt].listofpointsy[j],color=annot[cnt].colourID[j],marker='o')
            fw.draw()
        elif (ok == 'Rect'):
            ax.axis('equal')
            if (cnt>0) and (cnt<len(annot)):
                self.axes.plot([c1[0],c2[0]],[c1[1],c1[1]],'r')
                self.axes.plot([c2[0],c2[0]],[c1[1],c2[1]],'r')
                self.axes.plot([c2[0],c1[0]],[c2[1],c2[1]],'r')
                self.axes.plot([c1[0],c1[0]],[c2[1],c1[1]],'r')
                fw.draw()
        elif (ok == 'No'):
            self.axes.clear()
            fw.draw()

    def training(self):

        global annot,samex,samey,c1,c2,colorName,colours,colour_index,colourID,listofpointsx,listofpointsy,ok,firstclick,secondclick,cnt, bag_file, data,selections

        for i in range(len(annot[cnt].samex)):
            if ((annot[cnt].samex[i] >= c1[0]) and (annot[cnt].samex[i] <= c2[0]) and ((annot[cnt].samey[i] >= c2[1]) and (annot[cnt].samey[i] <= c1[1]))):
                colorName = colours[colour_index]
                annot[cnt].colourID.append(colorName)
                annot[cnt].listofpointsx.append(annot[cnt].samex[i])
                annot[cnt].listofpointsy.append(annot[cnt].samey[i])
                #annot[cnt].samex = [x for x in annot[cnt].samex if x not in annot[cnt].listofpointsx]
                #annot[cnt].samey = [y for y in annot[cnt].samey if y not in annot[cnt].listofpointsy]
        ok = 'Yes'
        laserGlobals.scan_widget.drawLaserScan()
        colour_index+=1
        if (colour_index == (len(colours))):
           colour_index = 0 
        firstclick = False
        secondclick = False

        #SAVE to CSV
        filename = bag_file.replace(".bag","_laser.csv")
        with open(filename, 'w') as data:
            write = csv.writer(data)
            for row in annot:
                row_ = [row.samex, row.samey, row.listofpointsx, row.listofpointsy, row.annotID, row.colourID]
                write.writerow(row_)
            data.close()



class ApplicationWindow(QtWidgets.QMainWindow):

    def __init__(self):

        global  classes, le

        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

        self.main_widget = QtWidgets.QWidget(self)

        scan_widget = LS(self.main_widget)

        #Define buttons
        scanLayout = QHBoxLayout() 
        scanLayout.addWidget(scan_widget)

        playButton = QPushButton("Play")
        pauseButton = QPushButton("Pause")
        prevFrameButton = QPushButton("Previous")
        nextFrameButton = QPushButton("Next")
        stopButton = QPushButton("Stop")
        le = QLineEdit(self)
        le.setDragEnabled(True)
        addButton = QPushButton('Add', self)

        classes = QComboBox()
        classes.addItem('Classes')

        buttonLayout = QHBoxLayout()
        buttonLayout.addWidget(playButton)
        buttonLayout.addWidget(pauseButton)
        buttonLayout.addWidget(prevFrameButton)
        buttonLayout.addWidget(nextFrameButton)
        buttonLayout.addWidget(stopButton)
        buttonLayout.setAlignment(Qt.AlignTop)

        classLayout = QVBoxLayout()
        classLayout.addWidget(classes)
        classLayout.addWidget(le)
        classLayout.addWidget(addButton)
        classLayout.setAlignment(Qt.AlignTop)

        layout = QVBoxLayout(self.main_widget)
        layout.addLayout(scanLayout)
        layout.addLayout(buttonLayout)
        layout.addLayout(classLayout)

        #Define Connections
        playButton.clicked.connect(self.bplay)
        pauseButton.clicked.connect(self.bpause)
        prevFrameButton.clicked.connect(self.bprevious)
        nextFrameButton.clicked.connect(self.bnext)
        stopButton.clicked.connect(self.bstop)
        classes.activated[str].connect(self.chooseClass)
        addButton.clicked.connect(self.showObject)

        fig.canvas.mpl_connect('button_press_event', self.onClick)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)

    def bplay(self):
        global scan_widget
        scan_widget.ptime()

    def bpause(self):
        global timer
        timer.stop()

    def bprevious(self):
        global cnt, ok, scan_widget
        if (cnt>0):
            cnt = cnt-1
            ok = 'Yes'
            scan_widget.drawLaserScan()
        else:
            ok = 'No'
            scan_widget.drawLaserScan()

    def bnext(self):
        global cnt, annot, ok, scan_widget,colour_index
        colour_index = 0
        if (cnt<len(annot)):
            cnt = cnt+1
            ok = 'Yes'
            scan_widget.drawLaserScan()
        else:
            ok = 'No'
            scan_widget.drawLaserScan()

    def bstop(self):
        global cnt,timer,ax,fw
        cnt = 0
        timer.stop()
        ax.clear()
        fw.draw()

    def onClick(self, event):

        global firstclick, c1, secondclick, c2, ok, scan_widget, thirdclick

        x = event.x
        y = event.y
        if event.button == Qt.LeftButton:
                if firstclick == False:
                    if event.inaxes is not None:
                        c1 = [event.xdata,event.ydata]
                        firstclick = True
                elif secondclick == False:
                    if event.inaxes is not None:
                        c2 = [event.xdata,event.ydata]
                        if (c2[0]<c1[0]):
                            temp_c = c2
                            c2 = c1
                            c1 = temp_c
                        secondclick = True
                        ok = 'Rect'
                        scan_widget.drawLaserScan()
                elif ((not thirdclick) and (secondclick)):
                    #if event.button == Qt.RightButton:
                        #na emfanizei to menu me tis classes 
                        #na exei kai tin epilogi pros9ikis class
                    if event.button == Qt.LeftButton:
                        ok = 'Yes'
                        firstclick = False
                        secondclick = False
                        if ((cnt>0) and (cnt<len(annot))):
                            scan_widget.drawLaserScan()

    def showObject(self):
        global le,selections,classes,objs,sel_pos
        if le.text() not in selections:
            classes.addItem(le.text())
            objs = le.text()
            selections.append(objs)

    def chooseClass(self,text):
        global scan_widget,txt,colour_index,colours,annot,selections
        txt = text
        if ((txt != 'Classes') and secondclick):
            annot[cnt].annotID.append(txt)
            colour_index = selections.index(txt)%(len(colours))
            scan_widget.training()

class laserAnn:

    global c1,c2, objx,objy, s1,s2, txt

    def __init__(self, samex_=None, samey_=None, listofpointsx_=None,listofpointsy_=None, annotID_=None, colourID_=None):

        self.samex = []
        self.samey = []
        self.listofpointsx = []
        self.listofpointsy = []
        self.annotID = []
        self.colourID = []

        if samex_ == None:
            self.samex = []
        else:
            self.samex = samex_
        if samey_ == None:
            self.samey = []
        else:
            self.samey = samey_
        if listofpointsx_ == None:
            self.listofpointsx = []
        else:
            self.listofpointsx = listofpointsx_
        if listofpointsy_ == None:
            self.listofpointsy = []
        else:
            self.listofpointsy = listofpointsy_
        if annotID_ == None:
            self.annotID = []
        else:
            self.annotID = annotID_
        if colourID_ == None:
            self.colourID = []
        else:
            self.colourID = colourID_

def run(laserx,lasery,bagFile, filename):

    global timer,annot,s1,s2,bag_file,colorName

    timer = QtCore.QTimer(None)

    bag_file = bagFile
    filename = filename.replace(".bag", "_laser.csv")
    if os.path.isfile(filename):
        with open(filename, 'rb') as data:
            if os.path.getsize(filename)>1:
                read = csv.reader(data)
                for row in read:

                    row[0] = row[0][1:-1]
                    row[1] = row[1][1:-1]
                    row[2] = row[2][1:-1]
                    row[3] = row[3][1:-1]
                    row[4] = row[4][1:-1]
                    row[5] = row[5][1:-1]
                    row[5] = row[5].replace("'","")
                    row[0] = row[0].split(", ")
                    row[1] = row[1].split(", ")
                    row[2] = row[2].split(", ")
                    row[3] = row[3].split(", ")
                    row[4] = row[4].split(", ")
                    row[5] = row[5].split(", ")

                    for i in range(0, len(row[0])):
                        if row[0][i] == "":
                            row[0] = []
                            break
                        else:
                            row[0][i] = float(row[0][i])
                    for i in range(0, len(row[1])):
                        if row[1][i] == "":
                            row[1] = []
                            break
                        else:
                            row[1][i] = float(row[1][i])
                    for i in range(0, len(row[2])):
                        if row[2][i] == "":
                            row[2] = []
                            break
                        else:
                            row[2][i] = float(row[2][i])
                    for i in range(0, len(row[3])):
                        if row[3][i] == "":
                            row[3] = []
                            break
                        else:
                            row[3][i] = float(row[3][i])
                    for i in range(0, len(row[4])):
                        if row[4][i] == "":
                            row[4] = []
                            break
                        else:
                            row[4][i] = str(row[4][i])
                    for i in range(0, len(row[5])):
                        if row[5][i] == "":
                            row[5] = []
                            break
                        else:
                            row[5][i] = str(row[5][i])

                    la = laserAnn(row[0], row[1], row[2], row[3], row[4], row[5])
                    annot.append(la)
    else:
        for i in range(len(laserx)):
            s1 = laserx[i].tolist()
            s2 = lasery[i].tolist()
            la = laserAnn(samex_=s1,samey_=s2)
            annot.append(la)

    #qApp = QtWidgets.QApplication(sys.argv)

    #s = ApplicationWindow()

    #s.show()

    #sys.exit(qApp.exec_())