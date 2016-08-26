from __future__ import unicode_literals
import sys
import os
import os.path
import random
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor

matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QFont, QPainter
from PyQt5.QtCore import Qt, QUrl, pyqtSignal, QFile, QIODevice, QObject, QRect
from PyQt5.QtMultimedia import (QMediaContent,
        QMediaMetaData, QMediaPlayer, QMediaPlaylist, QAudioOutput, QAudioFormat)
from PyQt5.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget, QToolTip, QLabel, QFrame, QGridLayout, QMenu, qApp, QLineEdit)


from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from numpy import arange, sin, pi

import wave
import numpy as np
import subprocess
import csv
import cv2
from pyAudioAnalysis import audioFeatureExtraction as aF
from pyAudioAnalysis import audioSegmentation as aS
from pyAudioAnalysis import audioBasicIO

from audioGlobals import audioGlobals
annotationColors = (['Speech', 'green'],['Music','red'], ['Activity', 'magenta'],['Laugh', 'yellow'])

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#  ONCLICK FUNCTION                                                                       #
#  count clicks on Waveform                                                               #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

def onclick(event):
    global annotationColors

    audioGlobals.tempPass = False
    flagDraw = True
    color = 'turquoise'

    if event.xdata != None:

        #Left Mouse Click -> select audio segment\
        #----------------------
        if event.button == Qt.LeftButton:
            # >> Get Clicks for Start-End Time...
            x = event.xdata
            # >> Clear axes...
            audioGlobals.fig.clear()

            # >> First Click
            if audioGlobals.counterClick == 1:
                audioGlobals.xStart = x*1000
                print 'Start time %f ms' %audioGlobals.xStart      
                audioGlobals.durationFlag = 1
                if audioGlobals.annotationFlag == True:
                    audioGlobals.tempPass = True
                    audioGlobals.annotationFlag = False
                
                audioGlobals.playerStarted = False
                
                # >> Start not out of waveform bounds
                if audioGlobals.xStart < 0:
                    audioGlobals.xStart = 0
                    print '-> Correction time %f ms' %audioGlobals.xStart

            # >> Second Click
            if audioGlobals.counterClick == 2 and audioGlobals.tempPass == False:
                if audioGlobals.isBold:
                    audioGlobals.isBold = False
                    audioGlobals.durationFlag = 0
                    audioGlobals.fig.clear()
                else:
                    audioGlobals.xEnd = x*1000
                    print 'End Time %f ms' %audioGlobals.xEnd
                    audioGlobals.durationFlag = 2
                    #check xStart < xEnd
                    if audioGlobals.xStart > audioGlobals.xEnd:
                        temp = audioGlobals.xStart
                        audioGlobals.xStart = audioGlobals.xEnd
                        audioGlobals.xEnd = temp
                        print '  '
                        print 'SWAP START-END TIME'
                        print '-------------'
                        print 'Start Time %f ms' %audioGlobals.xStart
                        print 'End Time %f ms' %audioGlobals.xEnd
                        print '-------------'
                audioGlobals.playerStarted = False


            audioGlobals.startTimeToPlay = audioGlobals.xStart
            audioGlobals.endTimeToPlay = audioGlobals.xEnd

            
            audioGlobals.counterClick = audioGlobals.counterClick + 1
            playFlag = False
            audioGlobals.fig.drawCursor(audioGlobals.xStart, audioGlobals.xEnd, color, playFlag)
            audioGlobals.fig.draw()
        else:
            #get mouse coords to specify delete widget position
            #----------------------
            audioGlobals.xPos = event.x
            audioGlobals.xCheck = event.xdata
            audioGlobals.xCheck = audioGlobals.xCheck * 1000

            audioGlobals.fig.clear()
            #Check for existing annotation
            print '=========================='
            print 'Selected segment'
            if not audioGlobals.selected:
                for index in range(len(audioGlobals.annotations)):
                    if audioGlobals.xCheck >= audioGlobals.annotations[index][0] and audioGlobals.xCheck <= audioGlobals.annotations[index][1]:
                        audioGlobals.startTimeToPlay = audioGlobals.annotations[index][0]
                        audioGlobals.endTimeToPlay = audioGlobals.annotations[index][1]
                        for colorIndex in range(len(annotationColors)):
                            if annotationColors[colorIndex][0] == audioGlobals.annotations[index][2]:
                                color = annotationColors[colorIndex][1]
            
                            elif audioGlobals.annotations[index][2][:8] == 'Speech::':
                                for shadeIndex in range(len(audioGlobals.shadesAndSpeaker)):
                                    if audioGlobals.annotations[index][2] == audioGlobals.shadesAndSpeaker[shadeIndex][0]:
                                        color = audioGlobals.shadesAndSpeaker[shadeIndex][1]

                audioGlobals.tempPass = True
                audioGlobals.durationFlag = 2
                playFlag = False
                audioGlobals.playerStarted = False
                audioGlobals.fig.drawCursor(audioGlobals.startTimeToPlay, audioGlobals.endTimeToPlay, color, playFlag)
                #fig.draw()
            else:
                audioGlobals.tempPass = True
                audioGlobals.durationFlag = 2
                audioGlobals.counterClick = audioGlobals.counterClick + 1
                playFlag = False
                audioGlobals.fig.drawCursor(audioGlobals.startTimeToPlay, audioGlobals.endTimeToPlay, color, playFlag)
                #fig.draw()
                audioGlobals.selected = False

            xS = audioGlobals.startTimeToPlay
            xE = audioGlobals.endTimeToPlay

            print 'Start time %f ms' %xS 
            print 'End time %f ms' %xE 
            print '=========================='
            #check for selected segment
            if audioGlobals.xCheck >= xS and audioGlobals.xCheck <= xE:
                audioGlobals.fig.drawBold(color)
                audioGlobals.fig.draw()
                audioGlobals.fig.annotationMenu()
                audioGlobals.fig.draw()
