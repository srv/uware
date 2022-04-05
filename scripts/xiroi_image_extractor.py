#!/usr/bin/env python3

import roslib
import rospy
import os 
import shutil #for file management, copy file
import rosbag, sys, csv
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix 
import nav_msgs 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from cola2_msgs.msg import NavSts
import datetime
import csv
import subprocess
import shutil

import cv2
import matplotlib
import numpy
import message_filters
import numpy as np
from cv_bridge import CvBridge
from math import pi,tan
import tf
import geometry_msgs.msg
from cola2_lib.utils.ned import NED
from osgeo import gdal
import osr
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from shapely.geometry import Polygon

class xiroi_img_extractor:

    def __init__(self, name):
        """ Init the class """
        # Initialize some parameters

        print("XIROI IMG EXTRACTOR_started !!!!")
        dir = rospy.get_param('~saving_dir', default='~home/bagfiles/xiroi_imgs/')
        self.dir_save_bagfiles = "/DATA_C/extracted_images/INVHALI/2020_09_17/15_46_41/"
        # self.dir_save_bagfiles = dir + 'cv2_extracted/'

        if not os.path.exists(self.dir_save_bagfiles):
            os.makedirs(self.dir_save_bagfiles)

        # img_topic="/xiroi/stereo_ch3/right/image_raw"
        img_topic="/stereo_down/left/image_rect_color"

        rospy.Subscriber(img_topic, Image, self.img_callback)

        print("IMAGE_TOPIC: ",img_topic)

        self.par = 0
   
    def img_callback(self,img_sub):

        if (self.par % 3) == 0:
            self.image_secs = img_sub.header.stamp.secs
            self.image_seq = img_sub.header.seq
            bridge = CvBridge()
            # cv_image = bridge.imgmsg_to_cv2(img_sub, desired_encoding="bgr8") #
            cv_image = bridge.imgmsg_to_cv2(img_sub)
            filename_suffix='JPG'
            filename=os.path.join(self.dir_save_bagfiles+str(self.image_seq) + "." + filename_suffix)
            cv2.imwrite(filename,cv_image)
            print("img saved as ", filename)
        
        self.par += 1
 
if __name__ == '__main__':
    try:
        rospy.init_node('xiroi_img_extractor')
        image_georeferencer = xiroi_img_extractor(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass