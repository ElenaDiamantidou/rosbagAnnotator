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

def buffer_data(bag, input_topic, compressed):
    image_buff = []
    time_buff  = []
    start_time = None
    bridge     = CvBridge()
    #bag = rosbag.Bag(bagFile)
    #Buffer the images, timestamps from the rosbag

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

def runMain(bagFileName, fileName):
#if __name__ == '__main__':
    #bagFileName = 'ss1_lsN_sc1A_ruedia_cg_v.bag'
    #bagFileName = rosbag.Bag(bagFileName)

    depthFileName = fileName.replace(".bag","_DEPTH.avi")
    if os.path.isfile(depthFileName):
        print colored('Load depth Video', 'yellow')
        return depthFileName
    else:
        print colored('Get depth data from ROS', 'green')
        (message_count,duration,compressed, framerate) = depth_bag_file(bagFileName, "/camera/depth/image_raw")
        (imageBuffer, time_buff) = buffer_data(bagFileName, "/camera/depth/image_raw", compressed)

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


