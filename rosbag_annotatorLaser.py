#!/usr/bin/env python
import roslib
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
import signal
import os
import sys
import time
import threading
import rosbag
import yaml
import numpy as np
import matplotlib.pyplot as plt
import argparse
import textwrap
import math
#import qt_laserscan
import ls_gui


programmName = os.path.basename(sys.argv[0])
laserDistances = []
theta = []
sx = []
sy = []

def parse_arguments():
	parser = argparse.ArgumentParser(
		prog='PROG',
		formatter_class=argparse.RawDescriptionHelpFormatter,
		description=textwrap.dedent('''\
		This script annotates a rosbag file and creates a result file,
		the following keys can be used for annotation and control
		\tEsc: Quits
		\ta: Go back 1 frame
		\td: Go forward 1 frame
		\tz: Writes the timestamp on the result file with id 4
		\te: Writes the timestamp on the result file with id 3
		\tq: Writes the timestamp on the result file with id 2
		\tw: Writes the timestamp on the result file with id 1
		\ts: Writes the timestamp on the result file with id 0
		\tspace: Pause image
		\t<-: Reduce playback speed
		\t->: Increase playback speed
		'''))
	parser.add_argument('-i', '--input-file',    required=True,  nargs='?', help="rosbag file, absolute path")
	parser.add_argument('-st', '--scan-topic', required=True,  nargs='?', help="topic to be used for scan annotation, e.g. /scan")
	parser.add_argument('-c', '--csv-file',  nargs='?', help="csv file with bounded boxes of the bag played")
	parser.add_argument('-o', '--output-file',  nargs='?', help="output annotation result file")
	parser.add_argument('-a', '--append', default=False, help="append result file instead of creating new", action='store_true')
	return parser.parse_args()

def play_bag_file(bag_file, csv_file):
	global laserDistances, sx, sy, theta

	compressed = False
	bag = rosbag.Bag(bag_file)
	info_dict = yaml.load(bag._get_yaml_info())
	topics =  info_dict['topics']
	topic = topics[1]
	messages =  topic['messages']
	duration = info_dict['duration']
	topic_type = topic['type']

	#Messages for test
	print "Script parameters: ","\n\t- Bag file: ", bag_file, "\n\t- Topic: ", input_topic, 
	print "\nRosbag topics found: "
	for top in topics:
		print "\t- ", top["topic"], "\n\t\t-Type: ", topic["type"],"\n\t\t-Fps: ", topic["frequency"]

	#Checking if the topic is compressed
	if 'CompressedImage' in topic_type:
		compressed = True
	else:
		compressed = False

	#Get framerate

	bridge = CvBridge()
	image_buff = []
	time_buff = []
	box_buff = []
	counter = 0
	buff_size = messages
	file_obj = open(feature_file, 'a')

	#Loop through the rosbag
	for topic, msg, t in bag.read_messages(topics=[input_topic]):
		#Get the scan
		laserDistances.append(np.array(msg.ranges))
		theta = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
		theta = np.degrees(theta)
		sx.append(np.cos(np.radians(theta)) * laserDistances[-1])
		sy.append(np.sin(np.radians(theta)) * laserDistances[-1])
	laserDistances = []
	bag.close()

if __name__ =='__main__':

	args = parse_arguments()
	bag_file = args.input_file
	csv_file = args.csv_file
	output_file = args.output_file
	input_topic = args.scan_topic
	append = args.append

	#Create results file
	if(output_file is None):
		feature_file = bag_file.split(".")[0].split("/")[-1] + "_RES"
	else:
		feature_file = output_file

	if os.path.exists(feature_file) and not append:
		os.remove(feature_file)

	print feature_file

	#Open bag and get framerate	
	play_bag_file(bag_file, csv_file)

	ls_gui.run(sx, sy, bag_file)
