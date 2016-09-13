#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""The Video Widget example shows how to implement a video widget
using QtMultimedia's QAbstractVideoSurface.

The following is a translation into PyQt5 from the C++ example found in
C:\QtEnterprise\5.1.1\msvc2010\examples\multimediawidgets\customvideosurface\customvideowidget."""
#from __future__ import unicode_literals

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
from operator import itemgetter
import itertools
import time

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from termcolor import colored

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

from matplotlib.widgets import Cursor
from numpy import arange, sin, pi
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.transforms as transforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

programmName = os.path.basename(sys.argv[0])

#input parameters
def parse_arguments():
    inputFile = sys.argv[-1]
    #print inputFile
    return inputFile

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
    #bag = rosbag.Bag(bagFile)
    #Buffer the images, timestamps from the rosbag

    i = 0

    for topic, msg, t in bag.read_messages(topics=[input_topic]):
        if start_time is None:
            start_time = t

        #Get the image
        if not compressed:
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "32FC1") # 16UC1
            except CvBridgeError as e:
                print e
        else:
            nparr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_GRAYSCALE)

        # normalize depth image 0 to 255
        depthImg = np.array(cv_image, dtype=np.float32)
        cv2.normalize(depthImg, depthImg, 0, 255, cv2.NORM_MINMAX)
        image_buff.append(depthImg)
        time_buff.append(t.to_sec() - start_time.to_sec())

        #sleep(0.1)
        # Update Progress Bar
        i += 1
        #printProgress(i, messages, prefix = 'Buffer Depth Data:', suffix = 'Complete', barLength = 50)

    return image_buff, time_buff


def depth_bag_file(bagFile, input_topic):
    topicKey = 0
    topic = 0
    flag = False
    bag = bagFile
    info_dict = yaml.load(bag._get_yaml_info())
    topics =  info_dict['topics']

    for key in range(len(topics)):
        if topics[key]['topic'] == input_topic:
            topicKey = key

    topic = topics[topicKey]
    messages =  topic['messages']
    duration = info_dict['duration']
    topic_type = topic['type']
    frequency = topic['frequency']


    #Checking if the topic is compressed
    if 'CompressedImage' in topic_type:
        compressed = True
    else:
        compressed = False

    #Get framerate
    framerate = messages/duration

    return messages,duration,compressed, framerate

def runMain(bagFileName, fileName,input_topic):
#if __name__ == '__main__':
    #bagFileName = 'ss1_lsN_sc1A_ruedia_cg_v.bag'
    #bagFileName = rosbag.Bag(bagFileName)

    depthFileName = fileName.replace(".bag","_DEPTH.avi")
    if os.path.isfile(depthFileName):
        print colored('Load depth Video', 'yellow')
        return depthFileName
    else:
        print colored('Get depth data from ROS', 'green')
        (message_count,duration,compressed, framerate) = depth_bag_file(bagFileName, input_topic)
        (imageBuffer, time_buff) = buffer_data(bagFileName, input_topic, compressed, message_count)
        print 'Write depth video...'
        #Check opencv version
        (major, _, _) = cv2.__version__.split(".")
        if major == '3':
            fourcc = cv2.VideoWriter_fourcc('X', 'V' ,'I', 'D')
        else:
            fourcc = cv2.cv.CV_FOURCC('X', 'V' ,'I', 'D')

        height, width = imageBuffer[0].shape

        # 0 for grayscale image
        # non zero values for color frames
        video_writer = cv2.VideoWriter(depthFileName, fourcc, framerate, (width,height), 0)

        if not video_writer.isOpened():
            self.errorMessages(2)
        else:
            #VideoWriter(const string& filename, int fourcc, double fps, Size frameSize, bool isColor=true)
            for frame in imageBuffer:
                depthFrame = frame.astype('uint8')
                video_writer.write(depthFrame)
            video_writer.release()


        return depthFileName


