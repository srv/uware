#!/usr/bin/env python3

# Python libs
import os
import csv
from math import pi, tan
import time

# OpenCV libs
import cv2

# ROS libs
import tf
# import tf2_ros
import rospy
from sensor_msgs.msg import Image
from cola2_msgs.msg import NavSts
from geometry_msgs.msg import PoseStamped
from cola2_lib.utils.ned import NED
import message_filters
from cv_bridge import CvBridge

class image_georeferencer:

    def __init__(self, name):
        """ Init the class """

        print("Getting parameters")
        self.max_altitude = rospy.get_param('~max_altitude', default = 5.5)
        self.overlap_threshold = rospy.get_param('~overlap_thrshld', default = 0.5)
        self.directory = rospy.get_param('~saving_dir', default = '/home/turbot/bagfiles')
        self.nskips = rospy.get_param('~nskips', default = 3)    #it just picks one image in every 3
        self.sensor_frame_id = rospy.get_param('~sensor_frame_id', default = "turbot/stereo_down/left_optical")
        self.world_frame_id = rospy.get_param('~world_frame_id', default = "world_ned")
        # FOV de la camera extret de excel de framerate i exposicio -> modificar
        # FOV-x water 34,73   deg         # FOV_x_water=34.73
        # FOV-y water 26,84   deg         # FOV_y_water=26.84
        FOV_x_water = rospy.get_param('~FOV_x_water', default = 34.73)
        FOV_y_water = rospy.get_param('~FOV_y_water', default = 26.84)

        if not "/" in self.directory[-1]:
            self.directory += "/"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        print("Node started with FOV_X_water, FOV_Y_water: ", FOV_x_water, FOV_y_water)
        print("Max altitude: ", self.max_altitude)
        print("Saving dir: ", self.directory)
        print("Sensor frame id: ", self.sensor_frame_id)
        print("World frame id: ", self.world_frame_id)

        self.rows = 0
        self.counter = 0
        self.FOV_x = 34.73 * ((2*pi)/360.0) # deg to rad
        self.FOV_y = 26.84 * ((2*pi)/360.0) # deg to rad
        self.use_transform = False
        self.listener = tf.TransformListener()

        #Subscribers
        nav_sub = message_filters.Subscriber("navigation", NavSts, queue_size=1)

        img_sub = message_filters.Subscriber("image_rect_color", Image, queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([nav_sub, img_sub], 250, 0.5, allow_headerless=True)
        ts.registerCallback(self.nav_image_update)


    def nav_image_update(self, nav_sub, img_sub):
        """ Navigation and image callback, saves img and a csv with img info """

        print("-------------------------------------------")
        print("Image: ", self.counter)

        # Get the NED
        if self.counter == 0:
            self.ned_origin_lat = nav_sub.origin.latitude
            self.ned_origin_lon = nav_sub.origin.longitude
            self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)
            time.sleep(0.1)
            if self.listener.canTransform(self.world_frame_id, self.sensor_frame_id, rospy.Time(0)):
                self.use_transform = True
                print("Use transform: ", self.use_transform)
            else:
                self.use_transform = False
                print("Use transform: ", self.use_transform)

        if (self.counter % self.nskips) == 0:

            print("NAV_IMAGE_UPDATE")

            if self.use_transform is True:
                pose_center_transformed, quaternion = self.listener.lookupTransform(self.world_frame_id, self.sensor_frame_id, rospy.Time(0))
                print("x_img_pos_trans: ", pose_center_transformed[0])
                print("y_img_pos_trans: ", pose_center_transformed[1])
                pose_center_lat, pose_center_lon, _ = self.ned.ned2geodetic([pose_center_transformed[0], pose_center_transformed[1], 0.0])
                print("CENTER LAT/LON TRANS: ", pose_center_lat, pose_center_lon)
            
            else:
                x_img_pos = nav_sub.position.north  
                y_img_pos = nav_sub.position.east 
                print("x_img_pos", x_img_pos)
                print("y_img_pos", y_img_pos)
                pose_center_lat, pose_center_lon, _ = self.ned.ned2geodetic([x_img_pos, y_img_pos, 0.0]) 
                print("CENTER LAT/LON: ", pose_center_lat, pose_center_lon)

            altitude = nav_sub.altitude
            print("altitude:", altitude)
            if altitude <= self.max_altitude: #filter emerging and submerging imgs
                self.export_to_csv(self.counter, pose_center_lat, pose_center_lon, altitude)
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(img_sub, desired_encoding = "bgr8")
                filename_suffix = 'JPG'
                filename = os.path.join(self.directory + str(self.counter) + "." + filename_suffix)
                cv2.imwrite(filename,cv_image)
                print("img saved as ", filename)
        
        self.counter += 1


    def export_to_csv(self, seq, lat, lon, z):
        header = ["#img_name", "latitude", "longitude", "altitude"]
        csv_file = os.path.join(self.directory + 'img_info.csv')
        print(csv_file)
        seq = str(seq) + ".JPG"
        data_list = [seq, lat, lon, z]
 
        with open(csv_file, 'a+') as file:

            writer = csv.writer(file, delimiter = ';')
            print("witing csv data: ", data_list)
            if self.rows == 0:
                writer.writerow(header)
            writer.writerow(data_list)
            self.rows += 1
            print("rows: ", self.rows)

        file.close()
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('image_georeferencer_no_tf')
        image_georeferencer = image_georeferencer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass