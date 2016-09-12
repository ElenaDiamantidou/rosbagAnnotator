#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.
The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""

import csv
import yaml
import cv2
import os
import rosbag
import argparse
import textwrap
import rospy
import json
import random
import matplotlib
import math
import time

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *
from PyQt5.QtMultimediaWidgets import *
import warnings
import itertools
import ast

from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from termcolor import colored


''''''''''''''''''''''''''''''''''''
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QFont, QPainter
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice, QObject, QRect
from PyQt5.QtMultimedia import (QMediaContent,
        QMediaMetaData, QMediaPlayer, QMediaPlaylist, QAudioOutput, QAudioFormat)
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget, QToolTip, QLabel, QFrame, QGridLayout, QMenu, qApp, QLineEdit)

from audioGlobals import audioGlobals
from laserGlobals import laserGlobals
import visualizeAudio as vA
import ganttChartAudio as gA
import graphicalInterfaceLaser as gL
from graphicalInterfaceAudio import ApplicationWindow
import rosbagAudio
import rosbagDepth
import rosbagLaser
import saveAudioSegments
''''''''''''''''''''''''''''''''''''

start_point = False
end_point = False
boxInitialized = False
annotationColors = ['#00FF00', '#FF00FF','#FFFF00','#00FFFF','#FFA500','#C0C0C0','#000000','#EAEAEA']
eventColors = ['#9fbf1f','#087649','#0a5b75','#181a8d','#7969b0','#76a9ea','#bef36e','#edfa84','#f18ed2','#753e20']
gantEnabled = False
posSlider = 0
xBoxCoord = []
#Declare the basic topics for the topic box
BasicTopics = ['Audio', 'Depth', 'Video' , 'Laser']

depthFileName = None
rgbFileName = None

def printProgress (iteration, total, prefix = '', suffix = '', decimals = 1, barLength = 100):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        barLength   - Optional  : character length of bar (Int)
    """
    formatStr       = "{0:." + str(decimals) + "f}"
    percents        = formatStr.format(100 * (iteration / float(total)))
    filledLength    = int(round(barLength * iteration / float(total)))
    bar             = '█' * filledLength + '-' * (barLength - filledLength)
    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),
    sys.stdout.flush()
    if iteration == total:
        sys.stdout.write('\n')
        sys.stdout.flush()

def buffer_data(bag, input_topic, compressed, messages):
    image_buff = []
    time_buff  = []
    start_time = None
    bridge     = CvBridge()

    i = 0

    #Buffer the images, timestamps from the rosbag
    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)

        image_buff.append(cv_image)
        time_buff.append(t.to_sec() - start_time.to_sec())

        i += 1
        #printProgress(i, messages, prefix = 'Buffer RGB Data:', suffix = 'Complete', barLength = 50)

    return image_buff,  time_buff

#Returns a buffer with boxes
def buffer_csv(csv_file):
    box_buff   = []
    metrics = []
    box_buff_action = []

    if csv_file is not None and os.path.exists(csv_file):
        with open(csv_file, 'r') as file_obj:
            csv_reader = csv.reader(file_obj, delimiter = '\t')
            row_1 = next(csv_reader)
            try:
                index = [x.strip() for x in row_1].index('Rect_id')
                if 'Class' not in row_1:
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+5)::])
                        box_buff.append((rec_id,x, y, width, height))
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
                else:
                    for row in csv_reader:
                        (rec_id,x, y, width, height) = map(int, row[index:index + 5])
                        (meter_X,meter_Y,meter_Z,top,meter_h,distance) = map(float, row[(index+6)::])
                        box_buff.append((rec_id,x, y, width, height))
                        if  isinstance(row[index+5],str):
                            string = row[index+5]
                            if string.startswith('[') and string.endswith(']'):
                                #Transform a string of list to list
                                string = ast.literal_eval(string)
                                box_buff_action.append(string)
                            else:
                                box_buff_action.append(string)
                        else:
                            box_buff_action.append(row[index+5])
                        metrics.append((meter_X,meter_Y,meter_Z,top,meter_h,distance))
            except:
                return False,False,False
            return box_buff,metrics,box_buff_action
    else:
        return False,False,False

def get_bag_metadata(bag):
    topicKey = 0
    topic = 0
    flag = False
    info_dict = yaml.load(bag._get_yaml_info())
    topics =  info_dict['topics']

    for key in range(len(topics)):
        if topics[key]['topic'] == '/camera/rgb/image_raw':
            topicKey = key

    topic = topics[topicKey]
    messages =  topic['messages']
    duration = info_dict['duration']
    topic_type = topic['type']
    frequency = topic['frequency']
    topics_List = []

    #Messages for test
    #print "\nRosbag topics found: "
    for top in topics:
        print "\t- ", top["topic"], "\n\t\t-Type: ", top["type"],"\n\t\t-Fps: ", top["frequency"]
        topics_List.append(top["topic"])
    topics_List = sorted(set(topics_List))
    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = messages/duration
    #framerate = 27

    return messages,duration,compressed, framerate, topics_List


class VideoWidgetSurface(QAbstractVideoSurface):

    def __init__(self, widget, parent=None):
        super(VideoWidgetSurface, self).__init__(parent)
        self.widget = widget
        self.imageFormat = QImage.Format_Invalid
        global frameCounter
        frameCounter = 0 #Frame Counter initialize

    def supportedPixelFormats(self, handleType=QAbstractVideoBuffer.NoHandle):
        formats = [QVideoFrame.PixelFormat()]
        if (handleType == QAbstractVideoBuffer.NoHandle):
            for f in [QVideoFrame.Format_RGB32, QVideoFrame.Format_ARGB32, QVideoFrame.Format_ARGB32_Premultiplied, QVideoFrame.Format_RGB565, QVideoFrame.Format_RGB555,QVideoFrame.Format_BGR24,QVideoFrame.Format_RGB24]:
                formats.append(f)
        return formats

    def isFormatSupported(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        _bool = False
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty() and _format.handleType() == QAbstractVideoBuffer.NoHandle):
            _bool = True
        return _bool

    def start(self, _format):
        imageFormat = QVideoFrame.imageFormatFromPixelFormat(_format.pixelFormat())
        size = _format.frameSize()
        #frameCounter = 0 #Frame Counter initialize
        if (imageFormat != QImage.Format_Invalid and not size.isEmpty()):
            self.imageFormat = imageFormat
            self.imageSize = size
            self.sourceRect = _format.viewport()
            QAbstractVideoSurface.start(self, _format)
            self.widget.updateGeometry()
            self.updateVideoRect()
            return True
        else:
            return False

    def stop(self):
        self.currentFrame = QVideoFrame()
        self.targetRect = QRect()
        QAbstractVideoSurface.stop(self)

        self.widget.update()

    def present(self, frame):
        global frameCounter,removeBool
        if (self.surfaceFormat().pixelFormat() != frame.pixelFormat() or self.surfaceFormat().frameSize() != frame.size()):
            self.setError(QAbstractVideoSurface.IncorrectFormatError)
            self.stop()
            return False
        else:
            self.currentFrame = frame
            frameCounter += 1
            removeBool = True #Removes the boxes on current frame
            self.widget.repaint(self.targetRect)
            return True

    def videoRect(self):
        return self.targetRect

    def updateVideoRect(self):
        size = self.surfaceFormat().sizeHint()
        size.scale(self.widget.size().boundedTo(size), Qt.KeepAspectRatio)
        self.targetRect = QRect(QPoint(0, 0), size);
        self.targetRect.moveCenter(self.widget.rect().center())

    def paint(self, painter):
        if (self.currentFrame.map(QAbstractVideoBuffer.ReadOnly)):
            oldTransform = painter.transform()
            if (self.surfaceFormat().scanLineDirection() == QVideoSurfaceFormat.BottomToTop):
                painter.scale(1, -1);
                painter.translate(0, -self.widget.height())

            image = QImage(self.currentFrame.bits(),
                    self.currentFrame.width(),
                    self.currentFrame.height(),
                    self.currentFrame.bytesPerLine(),
                    self.imageFormat
            )

            painter.drawImage(self.targetRect, image, self.sourceRect)
            painter.setTransform(oldTransform)

            self.currentFrame.unmap()

class VideoWidget(QWidget):

    def __init__(self, parent=None):
        global classLabels, imageBuffer
        super(VideoWidget, self).__init__(parent)
        self.setAutoFillBackground(False)
        self.setAttribute(Qt.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        palette = self.palette()
        palette.setColor(QPalette.Background, Qt.black)
        self.setPalette(palette)
        self.setSizePolicy(QSizePolicy.MinimumExpanding ,
        QSizePolicy.MinimumExpanding)
        self.surface = VideoWidgetSurface(self)
        self.vanishBox = False
        self.enableWriteBox = False
        self.annotEnabled = False
        self.annotClass = 'Clear'
        self.deleteEnabled = False
        self.deleteAllBoxes = False
        self.buttonLabels = []
        classLabels = []
        imageBuffer = []


    def videoSurface(self):
        return self.surface

    #Shows the right click menu
    def contextMenuEvent(self,event):
        global posX
        global posY
        global classLabels,gantChart,gantEnabled

        if event.reason() == QContextMenuEvent.Mouse:
            menu = QMenu(self)
            clear = menu.addAction('Clear')

            for i in classLabels:
                self.buttonLabels.append(menu.addAction(i))

            deleteBox = menu.addAction('Delete Box')
            deleteAllBoxes = menu.addAction('Delete All Boxes')
            changeId = menu.addAction('Change Id')
            cancel = menu.addAction('Cancel')
            action = menu.exec_(self.mapToGlobal(event.pos()))
            for i,key in enumerate(self.buttonLabels):
                if action == key:
                    self.annotClass = classLabels[i]
                    self.annotEnabled = True
            if action == deleteBox:
                self.deleteEnabled = True
            elif action ==  deleteAllBoxes:
                self.deleteAllBoxes = True
            elif action == changeId:
                #Call the textbox
                self.newBoxId = textBox()
                self.newBoxId.setGeometry(QRect(500, 100, 300, 100))
                self.newBoxId.show()
            elif action == cancel:
                pass
            elif action == clear:
                self.annotClass = 'Clear'
                self.annotEnabled = True

            self.posX_annot = event.pos().x()
            self.posY_annot = event.pos().y()

            posX = event.pos().x()
            posY = event.pos().y()

            self.repaint()
            self.buttonLabels = []
        self.annotEnabled = False

        #gantEnabled = True
        gantChart.axes.clear()
        gantChart.drawChart()
        gantChart.draw()

    def sizeHint(self):
        return self.surface.surfaceFormat().sizeHint()

    #Shows the video and bound boxes on it
    def paintEvent(self, event):
        global start_point
        global end_point
        global frameCounter
        global timeId

        painter = QPainter(self)
        rectPainter = QPainter(self)
        boxIdPainter = QPainter()

        if not rectPainter.isActive():
            rectPainter.begin(self)

        if (self.surface.isActive()):
            videoRect = QRegion(self.surface.videoRect())
            if not videoRect.contains(event.rect()):
                region = event.region()
                region.subtracted(videoRect)
                brush = self.palette().background()
                for rect in region.rects():
                    painter.fillRect(rect, brush)
            self.surface.paint(painter)
        else:
            painter.fillRect(event.rect(), self.palette().window())
        '''
        #If you press control and click, remove the clicked box from the list
        if player.controlEnabled :
            posX = self.eraseRectPos.x()
            posY = self.eraseRectPos.y()
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if posX > x and posX < (x+w) and posY > y and posY < (y+h):
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    timeId = player.videobox[frameCounter].timestamp[0]
                    player.videobox[frameCounter].removeBox() #CTRL + CLICK removes the box
        '''

        if self.deleteEnabled:
            i = 0
            while i < len(player.videobox[frameCounter].box_Id):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
                    timeId = player.videobox[frameCounter].timestamp[0]
                    player.videobox[frameCounter].removeSpecBox(player.videobox[frameCounter].box_Id[i])
                    for j in range(len(player.videobox[frameCounter].box_Id)):
                        x,y,w,h = player.videobox[frameCounter].box_Param[j]
                        if not rectPainter.isActive():
                            rectPainter.begin(self)
                        rectPainter.setRenderHint(QPainter.Antialiasing)
                        rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[j])))
                        rectPainter.drawRect(x,y,w,h)
                        rectPainter.end()

                        if not boxIdPainter.isActive():
                            boxIdPainter.begin(self)
                        boxIdPainter.setRenderHint(QPainter.Antialiasing)
                        boxIdPainter.setPen(QColor(255,0,0))
                        boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[j]))
                        boxIdPainter.end()
                i += 1
            self.deleteEnabled = False
        #Deletes all boxes in current framerate
        elif self.deleteAllBoxes:
            timeId = player.videobox[frameCounter].timestamp[0]
            for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setPen(Qt.red)
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
            player.videobox[frameCounter].removeAllBox()
            self.deleteAllBoxes = False
        #Enabled when annotating
        elif self.annotEnabled:
            self.frameNumber = frameCounter
            box = None
            for i in range(len(player.videobox[frameCounter].box_Id)):
                x,y,w,h = player.videobox[frameCounter].box_Param[i]
                if self.posX_annot > x and self.posX_annot < (x+w) and self.posY_annot > y and self.posY_annot < (y+h):
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(self.annotClass)))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()
                    player.videobox[frameCounter].changeClass(i,self.annotClass)
                    box = i
                else:
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

            #Annotate the box at remaining frames
            while self.frameNumber < len(player.time_buff):
                if box >= len(player.videobox[self.frameNumber].box_Id) or box is None:
                    break
                player.videobox[self.frameNumber].changeClass(box,self.annotClass)
                self.frameNumber += 1

        elif start_point is True and end_point is True:
                x = event.rect().x()
                y = event.rect().y()
                w = event.rect().width()
                h = event.rect().height()

                #Keep the timestamp to add the new box
                if  len(player.videobox[frameCounter].timestamp):
                    timeId = player.videobox[frameCounter].timestamp[0]

                if self.enableWriteBox:
                    boxNumber = len(player.videobox[frameCounter].box_Id)
                    #If id already in the list then give the next id
                    if boxNumber in player.videobox[frameCounter].box_Id:
                        boxNumber += 1
                    player.videobox[frameCounter].addBox(timeId,[boxNumber,x,y,w,h],'Clear')
                    self.enableWriteBox = False

                for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                            rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

        #Play the bound boxes from csv
        elif len(player.videobox) > 0 and frameCounter < len(player.time_buff):
                for i in range(len(player.videobox[frameCounter].box_Id)):
                    x,y,w,h = player.videobox[frameCounter].box_Param[i]
                    if not rectPainter.isActive():
                        rectPainter.begin(self)
                    rectPainter.setRenderHint(QPainter.Antialiasing)
                    rectPainter.setPen(QColor(self.getColorBox(player.videobox[frameCounter].annotation[i])))
                    rectPainter.drawRect(x,y,w,h)
                    rectPainter.end()

                    if not boxIdPainter.isActive():
                        boxIdPainter.begin(self)
                    boxIdPainter.setRenderHint(QPainter.Antialiasing)
                    boxIdPainter.setPen(QColor(255,0,0))
                    boxIdPainter.drawText(QRectF(x+2,y,w,h),Qt.AlignLeft,str(player.videobox[frameCounter].box_Id[i]))
                    boxIdPainter.end()

        if rectPainter.isActive():
            rectPainter.end()

    #Mouse callback handling Boxes
    def mousePressEvent(self,event):
        global start_point,end_point

        if player.controlEnabled and QMouseEvent.button(event) == Qt.LeftButton:
             self.eraseRectPos= QMouseEvent.pos(event)
             self.repaint()
        elif QMouseEvent.button(event) == Qt.LeftButton:
            if start_point is True and end_point is True:
                pass
            elif start_point is False:
                QPoint.pos1 = QMouseEvent.pos(event)
                start_point = True
            elif end_point is False:
                QPoint.pos2 = QMouseEvent.pos(event)
                rect = QRect(QPoint.pos1,QPoint.pos2)
                end_point = True
                self.repaint()
                self.enableWriteBox = True
                self.repaint(rect)

                start_point = False
                end_point = False

    def resizeEvent(self, event):
        QWidget.resizeEvent(self, event)
        self.surface.updateVideoRect()

    def getColorBox(self,action):
        global classLabels
        for index,key in enumerate(classLabels):
            if action == key:
                return annotationColors[index % len(annotationColors)]
            elif action == 'Clear':
                return '#0000FF'

class textBox(QWidget):

    def __init__(self):
        global frameCounter
        global posX,posY

        QWidget.__init__(self)
        self.setWindowTitle('Set Box id')
        self.main_widget = QWidget(self)
        self.boxId = QLineEdit(self)
        self.Ok = QPushButton("Ok", self)

    def paintEvent(self, event):
        self.boxId.setPlaceholderText('Box Id:')
        self.boxId.setMinimumWidth(100)
        self.boxId.setEnabled(True)

        self.boxId.move(90, 15)
        self.Ok.move(115, 60)

        self.boxId.textChanged.connect(self.boxChanged)
        self.Ok.clicked.connect(self.closeTextBox)

        self.Ok.show()
        self.boxId.show()

    def boxChanged(self,text):

        self.box_Idx = text

    def closeTextBox(self):
        try:
            self.box_Idx = int(self.box_Idx)
        except:
            msgBox = QMessageBox()
            msgBox.setText("Wrong type, integer expected")
            msgBox.resize(100,40)
            msgBox.exec_()

        #Check id
        for i in range(len(player.videobox[frameCounter].box_Id)):
            if self.box_Idx == player.videobox[frameCounter].box_Id[i]:
                #Box Id already given
                msgBox = QMessageBox()
                msgBox.setText("Box Id already given")
                msgBox.resize(100,40)
                msgBox.exec_()

        for i in range(len(player.videobox[frameCounter].box_Id)):
            x,y,w,h = player.videobox[frameCounter].box_Param[i]
            if posX > x and posX  < (x+w) and posY > y and posY < (y+h):
                old_value = player.videobox[frameCounter].box_Id[i]
                player.videobox[frameCounter].box_Id[i] = self.box_Idx
                self.writeEnable = True
                self.frameNumber = frameCounter
                old_index = i
                break

        if self.writeEnable:
            while self.frameNumber < len(player.time_buff):
                if old_value in player.videobox[self.frameNumber].box_Id:
                    player.videobox[self.frameNumber].box_Id[old_index] = self.box_Idx
                self.frameNumber += 1
            self.writeEnable = False

        self.Ok.clicked.disconnect()
        self.close()

#Class for Drop down boxes about topic selection
class TopicBox(QDialog):
    def __init__(self):
        super(TopicBox,self).__init__()
        global BasicTopics,Topics
        self.setWindowTitle('Select Topics')
        self.setGeometry(280, 260, 440, 400)
        self.move(QApplication.desktop().screen().rect().center()- self.rect().center())
        self.okButton = QPushButton("Ok", self)
        self.okButton.move(180,360)
        self.okButton.clicked.connect(self.close_window)
        self.okButton.setEnabled(False)

        self.okButtonPush = False
        self.topic_options = []

    def show_topics(self):
        x = 30
        y = 40
        self.dropDownBox = []
        self.temp_topics = []
        print '2.1'
        for index,topic in enumerate(BasicTopics):
            self.topic_options.append(QLabel(self))
            self.topic_options[index].move(x,y)
            self.topic_options[index].setText(BasicTopics[index])
            self.dropDownBox.append(QComboBox(self))
            y += 60
        print '2.2'
        x = 120
        y = 35
        for key,option in enumerate(self.dropDownBox):
            self.dropDownBox[key].addItem('Choose Topic')
            self.dropDownBox[key].addItems(Topics)
            self.dropDownBox[key].move(x, y)
            self.dropDownBox[key].currentTextChanged.connect(self.selectionchange)
            y += 60
        print '2.3'
        #initialize list
        for index in range(len(BasicTopics)) :
            self.temp_topics.append([index,'Choose Topic'])

        print '2.3'
        self.exec_()

    def selectionchange(self,text):
        topic_counter = 0
        for key,option in enumerate(self.dropDownBox):
            if self.dropDownBox[key].currentText() == 'Choose Topic':
                topic_counter += 1
        if topic_counter == len(BasicTopics):
            self.okButton.setEnabled(False)
        else:
            self.okButton.setEnabled(True)

        for key,option in enumerate(self.dropDownBox):
            if text == self.dropDownBox[key].currentText():
                ddbox_index = key

        if len(self.temp_topics) > 0:
            for idx,value in enumerate(self.temp_topics):
                if value[0] == ddbox_index:
                    self.temp_topics.pop(idx)
                    self.temp_topics.append([ddbox_index,str(text)])
            if [ddbox_index,text] not in self.temp_topics:
                self.temp_topics.append([ddbox_index,str(text)])
        else:
            self.temp_topics.append([ddbox_index,str(text)])

    def close_window(self):
        #Sort by its first element
        self.temp_topics.sort(key=lambda x: x[0])
        self.okButtonPush = True
        print self.temp_topics
        self.close()

    def closeEvent(self,event):
        if not self.okButtonPush:
            msgBox = QMessageBox()
            msgBox.setIcon(msgBox.Question)
            msgBox.setText("Program will exit, are you sure?")
            msgBox.resize(140,60)
            msgBox.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
            msgBox.setDefaultButton(QMessageBox.No)
            ret = msgBox.exec_()
            if ret == QMessageBox.Yes:
                self.close()
                player.close()
            elif ret == QMessageBox.No:
                event.ignore()


class VideoPlayer(QWidget):
    def __init__(self, parent=None):
        global gantChart, Topics
        super(VideoPlayer, self).__init__(parent)
        self.videobox = []
        self.time_ = 0
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #self.setWindowFlags(self.windowFlags() | QtCore.Qt.CustomizeWindowHint)
        #self.setWindowFlags( (self.windowFlags() | Qt.CustomizeWindowHint) & ~Qt.WindowMaximizeButtonHint)
        Topics = None

        self.box_buffer = []
        self.metric_buffer = []

        self.topic_window = TopicBox()
        # >> DEFINE WIDGETS OCJECTS
        # >> VIDEO - DEPTH - AUDIO - LASER - GANTT CHART
        #----------------------
        self.videoWidget = VideoWidget()
        self.laserScan = gL.LS()

        # >> LASER
        #Define Laser Buttons
        scanLayout = QHBoxLayout()
        scanLayout.addWidget(self.laserScan)
        # >> Video Gantt Chart
        self.gantt = gantShow()
        gantChart = self.gantt
        gantChart.axes.get_xaxis().set_visible(False)
        gantChart.setFixedSize(1300, 130)


        # >> Define Audio annotations and gantt chart
        #----------------------
        self.wave = vA.Waveform()
        audioGlobals.fig = self.wave
        self.wave.axes.get_xaxis().set_visible(False)
        self.wave.draw()
        self.wave.setFixedSize(1300, 185)

        self.chart = gA.Chart()
        audioGlobals.chartFig = self.chart
        self.chart.setFixedSize(1300, 95)
        playButtonLaser = QPushButton("Play")
        playButtonLaser.setFixedWidth(80)
        playButtonLaser.setFixedHeight(30)
        pauseButtonLaser = QPushButton("Pause")
        pauseButtonLaser.setFixedWidth(80)
        pauseButtonLaser.setFixedHeight(30)
        prevFrameButtonLaser = QPushButton("Previous")
        prevFrameButtonLaser.setFixedWidth(80)
        prevFrameButtonLaser.setFixedHeight(30)
        nextFrameButtonLaser = QPushButton("Next")
        nextFrameButtonLaser.setFixedWidth(80)
        nextFrameButtonLaser.setFixedHeight(30)
        stopButtonLaser = QPushButton("Stop")
        stopButtonLaser.setFixedWidth(80)
        stopButtonLaser.setFixedHeight(30)


        buttonLayoutLaser = QHBoxLayout()
        buttonLayoutLaser.addWidget(playButtonLaser)
        buttonLayoutLaser.addWidget(pauseButtonLaser)
        buttonLayoutLaser.addWidget(prevFrameButtonLaser)
        buttonLayoutLaser.addWidget(nextFrameButtonLaser)
        buttonLayoutLaser.addWidget(stopButtonLaser)
        buttonLayoutLaser.setAlignment(Qt.AlignLeft)


        #Define Connections
        playButtonLaser.clicked.connect(self.laserPlay)
        pauseButtonLaser.clicked.connect(self.laserPause)
        prevFrameButtonLaser.clicked.connect(self.laserPrevious)
        nextFrameButtonLaser.clicked.connect(self.laserNext)
        stopButtonLaser.clicked.connect(self.laserStop)

        # >> Set Fix Size at Video Widget and LaserScan
        # >> (Half Gui)
        self.laserScan.setFixedSize(640, 480)
        self.videoWidget.setFixedSize(640, 480)
        self.openButton = QPushButton("Open...")
        self.openButton.setFixedWidth(75)
        self.openButton.setFixedHeight(30)
        self.importCsv = QPushButton("Import CSV...")
        self.importCsv.setFixedWidth(85)
        self.importCsv.setFixedHeight(30)
        self.openButton.clicked.connect(self.openFile)
        self.importCsv.clicked.connect(self.openCsv)

        # >> most important play button
        #----------------------
        self.playButton = QPushButton()
        self.playButton.setFixedHeight(30)
        self.playButton.setEnabled(False)
        self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))
        self.playButton.clicked.connect(self.play)

        # >> radio button for Depth or RGB
        #----------------------
        self.rgbButton = QRadioButton("RGB")
        self.rgbButton.setChecked(True)
        self.rgbButton.toggled.connect(self.rgbVideo)

        self.depthButton = QRadioButton("Depth")
        self.depthButton.toggled.connect(self.depth)


        self.positionSlider = QSlider(Qt.Horizontal)
        #self.positionSlider.setRange(0, audioGlobals.duration)
        self.positionSlider.setMinimum(0)
        self.positionSlider.setMaximum(audioGlobals.duration)
        self.positionSlider.setTickInterval(1)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        #add label to slider about elapsed time
        self.label_tmp = '<b><FONT SIZE=3>{}</b>'
        self.timelabel = QLabel(self.label_tmp.format('Time: ' + str(audioGlobals.duration)))


        self.label = QHBoxLayout()
        self.label.addWidget(self.timelabel)
        self.label.setAlignment(Qt.AlignRight)

        self.controlLayout = QHBoxLayout()
        self.controlLayout.addWidget(self.openButton)
        self.controlLayout.addWidget(self.importCsv)
        self.controlLayout.addWidget(self.playButton)
        self.controlLayout.addWidget(self.rgbButton)
        self.controlLayout.addWidget(self.depthButton)
        self.controlLayout.setAlignment(Qt.AlignLeft)
        #self.controlLayout.addStretch(1)
        self.controlEnabled = False

        videoLayout = QVBoxLayout()
        #videoLayout.addStretch(1)
        videoLayout.addWidget(self.videoWidget)
        videoLayout.addLayout(self.controlLayout)

        self.controlLaser = QHBoxLayout()
        self.controlLaser.addLayout(buttonLayoutLaser)
        self.controlLaser.addLayout(self.label)



        # >> Define Audio Player buttons
        #----------------------
        playButtonAudio = QPushButton("Play")
        playButtonAudio.setFixedWidth(80)
        playButtonAudio.setFixedHeight(30)
        pauseButtonAudio = QPushButton("Pause")
        pauseButtonAudio.setFixedWidth(80)
        pauseButtonAudio.setFixedHeight(30)
        stopButtonAudio = QPushButton("Stop")
        stopButtonAudio.setFixedWidth(80)
        stopButtonAudio.setFixedHeight(30)


        # >> Define Audio layouts
        #----------------------
        waveLayout = QVBoxLayout()
        waveLayout.addWidget(self.wave)
        waveLayout.addWidget(self.chart)


        # >> Laser Buttons Layout
        #----------------------
        buttonLayoutAudio = QHBoxLayout()
        buttonLayoutAudio.addWidget(playButtonAudio)
        buttonLayoutAudio.addWidget(pauseButtonAudio)
        buttonLayoutAudio.addWidget(stopButtonAudio)
        buttonLayoutAudio.setAlignment(Qt.AlignLeft)


        laserClass = QHBoxLayout()
        laserClass.addLayout(scanLayout)
        #laserClass.setAlignment(Qt.AlignTop)

        layoutLaser = QVBoxLayout()
        #layoutLaser.addStretch(1)
        layoutLaser.addLayout(laserClass)
        layoutLaser.addLayout(self.controlLaser)
        #layoutLaser.setAlignment(Qt.AlignBottom)

        # >> Specify final layout align
        #----------------------
        laserAndVideoLayout = QHBoxLayout()
        laserAndVideoLayout.addLayout(videoLayout)
        laserAndVideoLayout.addLayout(layoutLaser)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(laserAndVideoLayout)
        mainLayout.addWidget(self.positionSlider)
        mainLayout.addWidget(self.gantt)
        mainLayout.addLayout(waveLayout)
        mainLayout.addLayout(buttonLayoutAudio)

        self.setLayout(mainLayout)

        self.mediaPlayer.setVideoOutput(self.videoWidget.videoSurface())
        self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)

        # >> Player Buttons
        #----------------------
        playButtonAudio.clicked.connect(self.audioPlay)
        pauseButtonAudio.clicked.connect(self.audioPause)
        stopButtonAudio.clicked.connect(self.audioStop)


    def pauseMedia(self):
        self.mediaPlayer.pause()
        self.Pause()

    # VIDEO SWITCH RGB <-> Depth
    #----------------------

    def rgbVideo(self, enabled):
        global rgbFileName
        global frameCounter

        if enabled:

            self. depthEnable = False
            self.rgbEnable = True
            position = self.mediaPlayer.position()
            self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(rgbFileName))))
            self.mediaPlayer.setPosition(position)
            self.mediaPlayer.play()
            if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                self.player.setPosition(position)
                self.audioPlay()
            if self.topic_window.temp_topics[3][1] != 'Choose Topic':
                self.laserPlay()
            self.playButton.setEnabled(True)

    def depth(self, enabled):
        global depthFileName
        global frameCounter

        if enabled:

            self.rgbEnable = False
            self.depthEnable = True
            position = self.mediaPlayer.position()
            if self.topic_window.temp_topics[1][1] != 'Choose Topic':
                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(depthFileName))))
                self.mediaPlayer.setPosition(position)
                self.mediaPlayer.play()
            if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                self.player.setPosition(position)
                self.audioPlay()
            if self.topic_window.temp_topics[3][1] != 'Choose Topic':
                self.laserPlay()
            self.playButton.setEnabled(True)

    # AUDIO PLAYER BUTTON FUNCTIONS

    # >> Play audio (whole signal or segment)
    #----------------------
    def audioPlay(self):

        #GET CLICKS FROM WAVEFORM
        #----------------------
        #Initialize connection-position ONCE
        if not audioGlobals.playerStarted:
            #10ms for changePosition -> Not Delaying
            self.player.positionChanged.connect(self.checkPositionToStop)
            self.player.setNotifyInterval(10)
            if audioGlobals.durationFlag==0:
                audioGlobals.playerStarted = True
                audioGlobals.startTimeToPlay = 0
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==1:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.duration*1000 - 10
                audioGlobals.endTimeToPlay = self.end
                audioGlobals.counterClick = 3
            elif audioGlobals.durationFlag==2:
                audioGlobals.playerStarted = True
                self.start = audioGlobals.startTimeToPlay
                self.end = audioGlobals.endTimeToPlay
            self.player.setPosition(self.start)

        playFlag = True
        self.player.play()

    # >> Pause audio playing
    #----------------------
    def audioPause(self):
        #Not begging from self.start
        audioGlobals.playerStarted = True
        self.player.setPosition(self.time_)
        self.player.pause()

    # >> Stop audio playing
    #----------------------
    def audioStop(self):
        self.player.stop()
        #Begin again segment
        self.start = audioGlobals.startTimeToPlay
        self.player.setPosition(self.start)

    # >> Check ms in audio to stop play
    #----------------------
    def checkPositionToStop(self):
        self.time_ = self.player.position()
        #self.positionSlider.setValue(self.time_/1000)
        #print self.time_
        if self.time_ >= self.end:
            self.audioStop()
            self.player.setPosition(self.start)
            #self.positionSlider.setValue(self.start)


    # >> LASER BUTTON FUNCTIONS

    def laserPlay(self):
        self.laserScan.ptime()
        laserGlobals.scan_widget = self.laserScan

    def laserPause(self):
        laserGlobals.timer.stop()

    def laserPrevious(self):
        if (laserGlobals.cnt>0):
            laserGlobals.cnt = laserGlobals.cnt-1
            laserGlobals.ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
        else:
            laserGlobals.ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def laserNext(self):
        colour_index = 0
        if (laserGlobals.cnt<len(laserGlobals.annot)):
            laserGlobals.cnt = laserGlobals.cnt+1
            laserGlobals.ok = 'Yes'
            laserGlobals.scan_widget.drawLaserScan()
        else:
            laserGlobals.ok = 'No'
            laserGlobals.scan_widget.drawLaserScan()

    def laserStop(self):
        laserGlobals.cnt = 0
        laserGlobals.timer.stop()
        self.laserScan.axes.clear()
        self.laserScan.draw()

    def videoPosition(self):
        self.videoTime = self.mediaPlayer.position()

    '''
    def MyPopup(self):
        audioGlobals.msgDialog = QMessageBox()
        audioGlobals.msgDialog.setWindowTitle("Rosbag Data")

        #msg.setText("This is a message box")
        #msg.setInformativeText("This is additional information")
        #msg.setDetailedText("The details are as follows:")
        audioGlobals.msgDialog.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        retval = audioGlobals.msgDialog.exec_()
        print "value of pressed message box button:", retval
    '''

    def openFile(self):
        global imageBuffer,framerate
        global depthFileName, rgbFileName, Topics
        self.time_buff  = []
        start_time = None

        fileName,_ = QFileDialog.getOpenFileName(self, "Open Bag", QDir.currentPath(),"(*.bag)")

        # create a messsage box for get or load data info
        print not fileName
        if not fileName:
            pass
        else:
            try:
                bag = rosbag.Bag(fileName)
                (self.message_count,self.duration,compressed, framerate,Topics) = get_bag_metadata(bag)
                #Show window to select topics
                self.topic_window.show_topics()
            except:
                self.errorMessages(0)

            #Audio Handling
            if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                try:
                    audioGlobals.annotations = []
                    rosbagAudio.runMain(bag, str(fileName))
                except:
                    self.errorMessages(6)

            #Depth Handling
            if self.topic_window.temp_topics[1][1] != 'Choose Topic':
                depthFileName = rosbagDepth.runMain(bag, str(fileName),self.topic_window.temp_topics[1][1])
                try:
                    pass
                except:
                    self.errorMessages(7)

            #RGB Handling
            try:
                rgbFileName = fileName.replace(".bag","_RGB.avi")

                if os.path.isfile(rgbFileName):
                    print colored('Load RGB video', 'yellow')

                    # just fill time buffer in case that video exists
                    for topic, msg, t in bag.read_messages(topics=[self.topic_window.temp_topics[2][1]]):
                        if start_time is None:
                            start_time = t
                        self.time_buff.append(t.to_sec() - start_time.to_sec())
                else:
                    #Get bag metadata
                    print colored('Get rgb data from ROS', 'green')
                    (imageBuffer, self.time_buff) = buffer_data(bag,  self.topic_window.temp_topics[2][1], compressed, self.message_count)
                    rgb = rgbFileName.split('/')
                    print 'Write rgb video...'
                    #Check opencv version
                    #(major, _, _) = cv2.__version__.split(".")
                    #if major == '3':
                    #    fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
                    #else:
                    fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')

                    height, width, bytesPerComponent = imageBuffer[0].shape
                    video_writer = cv2.VideoWriter(rgbFileName, fourcc, framerate, (width,height), cv2.IMREAD_COLOR)

                    if not video_writer.isOpened():
                        self.errorMessages(2)
                    else:
                        #print("Video initialized")
                        for frame in imageBuffer:
                            video_writer.write(frame)
                        video_writer.release()
                    print colored('Finally...', 'yellow')
            except:
                self.errorMessages(8)

            #Laser Topic selection
            if self.topic_window.temp_topics[3][1] != 'Choose Topic':
                try:
                    rosbagLaser.runMain(bag, str(fileName),self.topic_window.temp_topics[3][1])
                    pass
                except:
                    self.errorMessages(9)

        self.wave.axes.clear()
        self.chart.axes.clear()
        try:
            if self.rgbButton:
                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(rgbFileName))))
                self.playButton.setEnabled(True)
            elif self.depthButton:
                self.mediaPlayer.setMedia(QMediaContent(QUrl.fromLocalFile(os.path.abspath(depthFileName))))
                self.playButton.setEnabled(True)

            #DEFINE PLAYER-PLAYLIST
            #----------------------
            self.source = QtCore.QUrl.fromLocalFile(os.path.abspath(audioGlobals.wavFileName))
            self.content = QMediaContent(self.source)
            self.player = QMediaPlayer()
            self.playlist = QMediaPlaylist(self)
            self.playlist.addMedia(self.content)
            self.player.setPlaylist(self.playlist)


            self.wave.drawWave()
            self.wave.drawAnnotations()
            self.wave.draw()

            self.chart.drawChart()
            self.chart.draw()

            self.setWindowTitle(fileName + ' -> Annotation')
        except:
            pass



    #Open CSV file
    def openCsv(self):
        global classLabels,gantEnabled
        self.box_buffer = []
        self.metric_buffer = []

        # OPEN VIDEO - DEPTH - AUDIO
        fileName,_ =  QFileDialog.getOpenFileName(self, "Open Csv ", QDir.currentPath(),"(*.csv)")
        box_buff,metrics_buff,box_action = buffer_csv(fileName)

        if not (box_buff or metrics_buff):
            self.errorMessages(1)
        else:
            self.box_buffer = [list(elem) for elem in box_buff]
            self.metric_buffer = [list(key) for key in metrics_buff]
            #Initialize objects which are equal to frames
            self.videobox = [boundBox(count) for count in range(len(self.time_buff))]

            #Frame counter initialize
            counter = -1
            if len(box_action)>0:
                self.box_actionBuffer = [key for key in box_action]
                for idx,key in enumerate(self.box_buffer):
                    if key[0] == 0:
                        counter += 1
                        self.videobox[counter].addBox(self.time_buff[counter],key,self.box_actionBuffer[idx])
                    else:
                        self.videobox[counter].addBox(self.time_buff[counter],key,self.box_actionBuffer[idx])
            else:
                for idx,key in enumerate(self.box_buffer):
                    if key[0] == 0:
                        counter += 1
                        self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')
                    else:
                        self.videobox[counter].addBox(self.time_buff[counter],key,'Clear')

            #Parse json file
            try:
                classLabels = self.parseJson()
            except:
                self.errorMessages(3)

            gantEnabled = True
            gantChart.axes.clear()
            gantChart.drawChart()
            gantChart.draw()

    def parseJson(self):
        with open("labels.json") as json_file:
                json_data = json.load(json_file)
                json_label = []
                for i in json_data['labels'] :
                    json_label.append(i)
        return json_label

    def errorMessages(self,index):
        msgBox = QMessageBox()
        msgBox.setIcon(msgBox.Warning)
        if index == 0:
            msgBox.setText("Error: Incorrect Bag File")
        elif index == 1:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Error occured: Please check CSV file")
        elif index == 2:
            msgBox.setIcon(msgBox.Critical)
            msgBox.setText("Error: Video could not initialized")
        elif index == 3:
            msgBox.setText("Error: Json file path error")
        elif index == 4:
            msgBox.setText("Not integer type")
        elif index == 5:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Box id already given")
        elif index == 6:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Incorrect Audio Topic")
        elif index == 7:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Incorrect Depth Topic")
        elif index == 8:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Incorrect RGB Topic")
        elif index == 9:
            msgBox.setIcon(msgBox.Warning)
            msgBox.setText("Incorrect Laser Topic")

        msgBox.resize(100,40)
        msgBox.exec_()

    def play(self):
        global frameCounter, posSlider, durationSlider
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.videoPosition()
            self.mediaPlayer.pause()
            if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                self.audioPause()
            if self.topic_window.temp_topics[3][1] != 'Choose Topic':
                self.laserPause()
            self.time_ = self.positionSlider

        else:
            self.time_ = self.mediaPlayer.position()
            if self.topic_window.temp_topics[0][1] != 'Choose Topic':
                self.player.setPosition(self.time_)
                self.end = audioGlobals.duration*1000 - 10
                self.audioPlay()
            if self.topic_window.temp_topics[2][1] != 'Choose Topic':
                self.mediaPlayer.play()
            if self.topic_window.temp_topics[3][1] != 'Choose Topic':
                self.laserPlay()

        # >> Get slider position for bound box
        posSlider = self.positionSlider.value()
        #self.tickLabel.setAlignment(posSlider)
        frameCounter = int(round(self.message_count * posSlider/(self.duration * 1000)))

    def mediaStateChanged(self, state):
        if state == QMediaPlayer.PlayingState:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        time = "{0:.2f}".format(float(position)/1000)
        self.positionSlider.setValue(position)
        self.positionSlider.setToolTip(str(time) + ' sec')
        self.timelabel.setText(self.label_tmp.format('Time: ' + str(time) + '/ ' + str("{0:.2f}".format(self.duration)) + ' sec'))
        laserGlobals.cnt = position/100

        #self.timelabel.(position)

    def keyPressEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = True

    def keyReleaseEvent(self,event):
        if event.key() == Qt.Key_Control:
            self.controlEnabled = False

    def durationChanged(self, duration):
        global durationSlider
        durationSlider = duration
        self.positionSlider.setRange(0, duration)


    def setPosition(self, position):
        global frameCounter, posSlider
        frameCounter = int(round(self.message_count * position/(self.duration * 1000)))
        posSlider = position
        if self.topic_window.temp_topics[2][1] != 'Choose Topic':
            self.mediaPlayer.setPosition(position)
        if self.topic_window.temp_topics[0][1] != 'Choose Topic':
            self.player.setPosition(position)

    #Writes the boxes to csv
    def writeCSV(self,videobox):
        list_insert_time = []
        list_insert_box = []
        list_insert_class = []
        list_insert_param_1 = []
        list_insert_param_2 = []
        list_insert_param_3 = []
        list_insert_param_4 = []
        list_metr_param_1 = []
        list_metr_param_2 = []
        list_metr_param_3 = []
        list_metr_param_4 = []
        list_metr_param_5 = []
        list_metr_param_6 = []

        for i in self.videobox:
            for j in i.timestamp:
                list_insert_time.append(j)
            for k in i.box_Id:
                list_insert_box.append(k)
            for l in i.box_Param:
                list_insert_param_1.append(l[0])
                list_insert_param_2.append(l[1])
                list_insert_param_3.append(l[2])
                list_insert_param_4.append(l[3])
            for key in i.annotation:
                list_insert_class.append(key)

        if len(self.metric_buffer) > 0:
            for metr in self.metric_buffer:
                list_metr_param_1.append(metr[0])
                list_metr_param_2.append(metr[1])
                list_metr_param_3.append(metr[2])
                list_metr_param_4.append(metr[3])
                list_metr_param_5.append(metr[4])
                list_metr_param_6.append(metr[5])

        with open('boxes_updated.csv', 'w') as file:
            csv_writer = csv.writer(file, delimiter='\t')
            headlines = ['Timestamp','Rect_id', 'Rect_x','Rect_y','Rect_W','Rect_H','Class','Meter_X','Meter_Y','Meter_Z','Top','Height' ,'Distance']
            csv_writer.writerow(headlines)
            rows = zip(list_insert_time,list_insert_box,list_insert_param_1,list_insert_param_2,list_insert_param_3,list_insert_param_4,list_insert_class,list_metr_param_1,list_metr_param_2,list_metr_param_3,list_metr_param_4,list_metr_param_5,list_metr_param_6)
            csv_writer.writerows(rows)

    def closeEvent(self,event):
        self.writeCSV(self.videobox)


#Holds the bound box parameters
class boundBox(object):
    def __init__(self,parent=None):
        global xBoxCoord

        super(boundBox, self).__init__()
        self.timestamp = []
        self.box_Id = []
        self.box_Param = []
        self.annotation = []

    def addBox(self,time,key,classify):
        self.timestamp.append(time)
        self.box_Id.append(key[0])
        self.box_Param.append(key[1:])
        self.annotation.append(classify)

        self.calcAngle()

    def removeAllBox(self):
        self.timestamp[:] = []
        self.box_Id[:] = []
        self.box_Param[:] = []
        self.annotation[:] = []

    def removeSpecBox(self,boxid):
        self.timestamp.pop(boxid)
        self.box_Id.pop(boxid)
        self.box_Param.pop(boxid)
        self.annotation.pop(boxid)

    def changeClass(self,boxid,classify):
        if boxid < len(self.annotation):
            self.annotation.pop(boxid)
        self.annotation.insert(boxid,classify)

    def calcAngle(self):
        # let's say that camera angle is 58 degrees..
        camAngle = 58
        camAngleRadians = math.radians(camAngle)
        imWidth = 640 #pixels

        for index in range(len(self.box_Param)):
            # CENTRALIZE camera and laser
            # xCamera, yCamera, zCamera <--> xLaser, yLaser, zLaser IN METERS
            # zCamera and zLaser doesn't matter

            xCamera = 0
            xLaser = 0

            # Convert meters to pixels
            # 1m = 3779.527559px ; 1px = 0.000265m
            xCamera = xCamera * 3779.527559
            xLaser = xLaser * 3779.527559
            diff = xLaser - xCamera

            z = (imWidth/2)/ sin(camAngleRadians/2)
            #Construct the axis of triangle
            MK = math.sqrt(pow(z,2) - pow(imWidth/2,2))
            x1 = self.box_Param[index][0] + diff
            x2 = self.box_Param[index][0] + self.box_Param[index][2] + diff

            startPoint = abs(x1 - (imWidth/2))
            x1Angle = math.atan(startPoint/MK)
            if x1-(imWidth/2) > 0:
                x1Angle = x1Angle + (camAngleRadians/2)
            else:
                x1Angle = (camAngleRadians/2) - x1Angle

            endPoint = abs(x2 - (imWidth/2))
            x2Angle = math.atan(endPoint/MK)
            if x2-(imWidth/2) > 0:
                x2Angle = x2Angle + (camAngleRadians/2)
            else:
                x2Angle = (camAngleRadians/2) - x2Angle


            #angle = abs(math.degrees(x2Angle - x1Angle))

            # angle to laser 270 degrees
            x1 = x1 + math.radians(105)
            x2 = x2 + math.radians(105)

            #rho = np.sqrt(x1**2 + self.box_Param[index][1]**2)
            #phi = np.arctan2(self.box_Param[index][1], x1)
            #print rho, phi
            #print math.degrees(x1Angle)#, math.degrees(x2Angle)




class videoGantChart(FigureCanvas):
    def __init__(self, parent=None,width=15,height=1,dpi=100):
        gantChart = Figure(figsize=(width, height), dpi=dpi)
        self.axes = gantChart.add_subplot(111)

        self.drawChart()

        FigureCanvas.__init__(self, gantChart)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def drawChart(self):
        pass

#Class for the gantChart
class gantShow(videoGantChart):
    #Plot the chart
    def drawChart(self):
        global imageBuffer, framerate
        global annotationColors
        global xTicks
        global classLabels,gantEnabled

        self.timeWithId = []
        self.tickY = []
        self.tickX = []
        self.boxAtYaxes = []
        self.axes.hlines(0,0,0)

        time_index = 0
        #X axis with 5 sec timestep
        for index in range(len(imageBuffer)):
            if index % int(round(framerate)) == 0:
                self.tickX.append(time_index)
                time_index += 1

        if gantEnabled:
            for box_index in player.videobox:
                for boxIdx in box_index.box_Id:
                    if boxIdx > len(box_index.box_Id):
                        break
                    self.boxAtYaxes.append([boxIdx,box_index.annotation[box_index.box_Id.index(boxIdx)]])
                    self.timeWithId.append([boxIdx,box_index.timestamp[box_index.box_Id.index(boxIdx)],box_index.annotation[box_index.box_Id.index(boxIdx)]])

            #Remove duplicates for the y axis
            temp_set = set(map(tuple,self.boxAtYaxes))
            self.boxAtYaxes = sorted(map(list,temp_set))

            for key in range(len(self.boxAtYaxes)):
                self.tickY.append(key)
            for index in range(len(self.timeWithId)):
                self.startTime,self.endTime = self.timeCalc(self.timeWithId,index)
                if self.timeWithId[index][1] == self.endTime:
                    self.color = self.getColor(self.timeWithId[index][2])
                    self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],self.timeWithId[index][2]]), self.startTime,self.endTime+(1/framerate),linewidth=8,color=self.color)
                self.color = self.getColor(self.timeWithId[index][2])
                self.axes.hlines(self.boxAtYaxes.index([self.timeWithId[index][0],self.timeWithId[index][2]]), self.startTime,self.endTime,linewidth=8,color=self.color)

        for tick in self.axes.yaxis.get_major_ticks():
            tick.label.set_fontsize(9)

        #self.axes.set_xticks(self.tickX)
        self.axes.set_xticklabels([])
        #align with audio gantt chart
        self.axes.set_xlim([-1,audioGlobals.duration + 1])
        self.axes.get_xaxis().set_visible(True)
        self.axes.set_yticks(self.tickY)
        self.axes.set_ylim([-1,len(self.boxAtYaxes)])
        self.axes.set_yticklabels(['<'+str(index[0])+'>::'+index[1] for index in self.boxAtYaxes])
        self.axes.grid(True)

    #Calculates the end time for each annotation to plot
    def timeCalc(self,time,curr):
        temp_class = time[curr][2]
        temp_id = time[curr][0]
        starttime = time[curr][1]
        endtime = time[curr][1]
        while temp_class in time[curr] and temp_id in time[curr]:
            endtime = time[curr][1]
            curr += 1
            if curr > len(time)-1:
                break
        return starttime,endtime

    #Calculates the color for the gantChart and bound Boxes
    def getColor(self,action):
        global classLabels
        for index,key in enumerate(classLabels):
            if action == key:
                return annotationColors[index % len(annotationColors)]
            elif action == 'Clear':
                return '#0000FF'


if __name__ == '__main__':
    os.system('cls' if os.name == 'nt' else 'clear')
    app = QApplication(sys.argv)

    player = VideoPlayer()
    #player.setFixedSize(1300,1025)
    player.show()

    app.exec_()
    csvFileName = audioGlobals.bagFile.replace(".bag","_audio.csv")
    if audioGlobals.saveAudio == False:
        saveAudioSegments.save(csvFileName, audioGlobals.wavFileName)

    #sys.exit(app.exec_())

