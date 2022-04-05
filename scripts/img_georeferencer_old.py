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

class image_georeferencer:

    def __init__(self, name):
        """ Init the class """
       
        # Initialize some parameters 
        self.overlap_threshold=rospy.get_param('~overlap_thrshld', default=0.5)

        scaling = rospy.get_param('~scaling', default=1)
        if scaling==1:
            img_topic="/stereo_down/right/image_rect_color"
        if scaling==2:
            img_topic="/stereo_down/scaled_x2/right/image_color"
        if scaling==8:
            img_topic="/stereo_down/scaled_x8/right/image_color"

        dir = rospy.get_param('~saving_dir', default='~home/bagfiles')
        self.dir_save_bagfiles = dir + 'CSV_TESTS/'

        #create directory 
        if not os.path.exists(self.dir_save_bagfiles):
            os.makedirs(self.dir_save_bagfiles)

        self.image_secs=None
        self.counter=0        
        self.skip_n_imgs=1    #save img every skip_n_imgs
        self.rows=0
        self.listener = tf.TransformListener()

        # FOV de la camera extret de excel de framerate i exposicio -> modificar
        # FOV-x water 34,73   deg         # FOV_x_water=34.73
        # FOV-y water 26,84   deg         # FOV_y_water=26.84
        FOV_x_water=rospy.get_param('~FOV_x_water',default=34.73)
        FOV_y_water=rospy.get_param('~FOV_y_water',default=26.84)

        print("Georeferencer started!! FOV_X_water, FOV_Y_water: ",FOV_x_water, FOV_y_water)

        #deg to rad
        self.FOV_x=34.73*((2*pi)/360.0)
        self.FOV_y=26.84*((2*pi)/360.0)

        #Subscribers
        gps_sub=message_filters.Subscriber("/turbot/navigator/navigation",
                        NavSts,  
                        queue_size=1)

        img_sub=message_filters.Subscriber(img_topic,
                         Image,
                         queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([gps_sub, img_sub], 10, 0.5, allow_headerless=True)
        ts.registerCallback(self.gps_image_update)

    
    # #get base_link to camera center tf 
    # def get_tf_transform(self):
    #     rot=None
    #     trans=None
    #     try:
    #         (trans,rot) = self.listener.lookupTransform('/turbot/base_link', '/turbot/stereo_down/left_optical', rospy.Time(0))
    #         print("trans!!",trans)
    #         print("rot!!!",rot)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print("No transformation available!")
    #     print(trans,"     ",rot)
    #     return trans,rot

    def get_overlapping(self,img_bounds1,img_bounds2):
        #Calculates the overlap ratio between 2 images
        # img_bounds1: poses of corners of img1 (left up,right up,right down,left down)
        # img_bounds2: poses of corners of img2 (left up,right up,right down,left down)
        # returns [overlap,overlap_proportion]
            # overlap: True if there is more overlap than the threshold, false otherwise
            # overlap proportion: overlap ratio of the 2 images

        bounds1=[]
        bounds2=[]
        # convert to (x,y) points
        for bound1,bound2 in zip(img_bounds1,img_bounds2):
            bounds1.append([bound1.pose.position.x,bound1.pose.position.y])
            bounds2.append([bound2.pose.position.x,bound2.pose.position.y])

        img1=Polygon([bounds1[0],bounds1[1],bounds1[2],bounds1[3]])
        img2=Polygon([bounds2[0],bounds2[1],bounds2[2],bounds2[3]])

        if img1.intersects(img2):
            overlap_area=img1.intersection(img2).area
            if img1.area <= img2.area:
                overlap_proportion=overlap_area/img1.area
            else:
                overlap_proportion=overlap_area/img2.area
            if overlap_proportion >= self.overlap_threshold:
                return True,overlap_proportion
            else:
                return False,None
        else:
            return False,None


    #navigation and image callback
    #gets image and localization info and saves it to a csv
    def gps_image_update(self,gps_sub,img_sub):

        if self.counter==0:
            # self.trans,self.rot=self.get_tf_transform()
            # print("trans!!",self.trans)
            self.ned_origin_lat=gps_sub.origin.latitude
            self.ned_origin_lon=gps_sub.origin.longitude
            self.ned = NED(self.ned_origin_lat, self.ned_origin_lon, 0.0)
            
        print("GPS_IMAGE_UPDATE")
        print("----------------------------------------------")
        print("COUNTER:          = ",self.counter)
        print(" ")
        self.position_secs = gps_sub.header.stamp.secs
        self.latitude = gps_sub.global_position.latitude
        self.longitude = gps_sub.global_position.longitude
        self.altitude = gps_sub.altitude
        print("lat, long, alt:",self.latitude,"   ",self.longitude," ",self.altitude)

        self.image_secs = img_sub.header.stamp.secs
        self.image_seq = img_sub.header.seq
        print("GPS_time_stamp: ",self.position_secs)
        print("image_time_stamp: ",self.image_secs)

        #north (x) east (y)
        x_img_pos=gps_sub.position.north #+ self.trans[0]
        y_img_pos=gps_sub.position.east #+ self.trans[1]
        print("x_img_pos",x_img_pos)
        print("y_img_pos",y_img_pos)

        #Image dimensions
        #tan function uses rad      
        self.x_dim_img=2*self.altitude*tan(self.FOV_x/2)#width
        self.y_dim_img=2*self.altitude*tan(self.FOV_y/2)#height
        print("IMG_DIM: ",self.x_dim_img,"        ",self.y_dim_img)

        lat_center, long_center, _ = self.ned.ned2geodetic([x_img_pos, y_img_pos, 0.0])
        print("CENTER_LAT_LONG__1: ",lat_center, long_center)
        print(" ")

        #Pose respect to img:
        pose_center = PoseStamped()
        pose_center.pose.position.x=0
        pose_center.pose.position.y=0

        pose_corner_r_up = PoseStamped()
        pose_corner_l_up = PoseStamped()
        pose_corner_r_down = PoseStamped()
        pose_corner_l_down = PoseStamped()

        corner_list=[pose_corner_l_up, pose_corner_r_up, pose_corner_r_down,pose_corner_l_down,pose_center]

        #Corners of img referenced to img_center:
        pose_corner_r_up.pose.position.x= self.x_dim_img/2
        pose_corner_r_up.pose.position.y= self.y_dim_img/2

        pose_corner_l_up.pose.position.x= -self.x_dim_img/2
        pose_corner_l_up.pose.position.y=  self.y_dim_img/2

        pose_corner_r_down.pose.position.x=  self.x_dim_img/2
        pose_corner_r_down.pose.position.y= -self.y_dim_img/2

        pose_corner_l_down.pose.position.x= -self.x_dim_img/2
        pose_corner_l_down.pose.position.y= -self.y_dim_img/2
   
        #lat,lon diff ->used only to georeference img
        # pxl_lat_up0, pxl_lon_up0, _ = self.ned.ned2geodetic([-self.x_dim_img/2, self.y_dim_img/2, 0.0])
        # pxl_lat_down0, pxl_lon_down0, _ = self.ned.ned2geodetic([self.x_dim_img/2, -self.y_dim_img/2, 0.0])
        # lat_diff = abs(pxl_lat_down0 - pxl_lat_up0) 
        # lon_diff = abs(pxl_lon_down0 - pxl_lon_up0) 

        self.img_corners=[] #
        #Convert corner's pose to a pose referenced to world
        for corner in corner_list:
            corner.header=Header(stamp=img_sub.header.stamp, frame_id='/turbot/stereo_down/left_optical')
            corner.pose.position.z= 0   #self.altitude
            corner.pose.orientation.x = 0
            corner.pose.orientation.y = 0
            corner.pose.orientation.z = 0
            corner.pose.orientation.w = 0
            corner_transformed=self.listener.transformPose("/world_ned", corner)
            self.img_corners.append(corner_transformed) #
            print("CORNERS")
            print(corner_transformed.pose)

        center_lat,center_lon,_ =self.ned.ned2geodetic([self.img_corners[4].pose.position.x, self.img_corners[4].pose.position.y, 0.0])
        self.img_info=[self.image_seq ,center_lat,center_lon,self.altitude]
        
        if self.counter==0:
            self.prev_img_corners=self.img_corners[:]
            self.prev_img_info=self.img_info[:]

        else:
            is_overlaping,overlap = self.get_overlapping(self.img_corners[0:-1],self.prev_img_corners[0:-1])
            print("OVERLAP: ",overlap )

            if is_overlaping==False:
            #The last image that overlaped enough was the last one so we save it
                print("Saving img!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!, overlap= ", overlap)
                
                # Transform to lat lon pxl right up and pixel left down
                # pxl_lat_up, pxl_lon_up, _ = self.ned.ned2geodetic([self.prev_img_corners[0].pose.position.x, self.prev_img_corners[0].pose.position.y, 0.0])
                # pxl_lat_down, pxl_lon_down, _ = self.ned.ned2geodetic([self.prev_img_corners[2].pose.position.x, self.prev_img_corners[2].pose.position.y, 0.0])
                # print("LC_LAT_LONG: ",pxl_lat_up,"  ", pxl_lon_up)
                # print(" ")
                # print("RC_LAT_LONG: ",pxl_lat_down," ",pxl_lon_down)
                # print(" ")
                # bounds=[pxl_lat_up, pxl_lon_up,pxl_lat_down, pxl_lon_down]

                #convert to CVimage
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(img_sub, desired_encoding="bgr8")
                filename_suffix='JPG'
                img_seq=self.prev_img_info[0]
                filename=os.path.join(self.dir_save_bagfiles+str(img_seq) + "." + filename_suffix)

                cv2.imwrite(filename,cv_image)
                print("img saved as ", filename)
                print("IMG shape")
            
                #####################SAVING CSV #################################################
                #prev_img_info:[self.image_seq,lat_center,long_center,self.altitude]
                self.export_to_csv(self.prev_img_info[0], self.prev_img_info[1], self.prev_img_info[2],self.prev_img_info[3])
                ##############################################################################
                
                #georeference:
                # self.geotiff(filename,cv_image,bounds,lat_diff,lon_diff)
                
                self.prev_img_corners=self.img_corners[:]
                self.prev_img_info=self.img_info[:]

        self.counter+=1

    def export_to_csv(self, seq, lat, long,z):
        print("WRITING_CSV")
        header=["img_name","latitude","longitude","altitude"]

        csv_file=os.path.join(self.dir_save_bagfiles+'img_info_test0.csv')
        seq=str(seq)+".JPG"
        data_list = [seq,lat,long,z]
        print("witing data: ",data_list,"in file: ",csv_file)

        with open(csv_file, 'a+') as file:
            writer = csv.writer(file, delimiter=';')
            if self.rows==0:
                writer.writerow(header)

            writer.writerow(data_list)
            self.rows+=1
            print("rows: ",self.rows)

        file.close()

    # adfGeoTransform[0] /* top left x */
    # adfGeoTransform[1] /* w-e pixel resolution */ ->xres lon
    # adfGeoTransform[2] /* 0 */
    # adfGeoTransform[3] /* top left y */
    # adfGeoTransform[4] /* 0 */
    # adfGeoTransform[5] /* n-s pixel resolution (negative value) */ ->yres lat

    def geotiff(self,filename, image,bounds,lat_diff,lon_diff):

        print("filename: ",filename)
        ny=image.shape[0] #img height 1440
        nx=image.shape[1] #img width 1920
        xres = lon_diff / float(nx) 
        yres = lat_diff / float(ny)

        geotransform = (bounds[0], xres, 0, bounds[1], 0, yres)
        # To work in qgis(lat lon must be lon lat):
        # geotransform = (bounds[1], xres, 0, bounds[0], 0, yres)

        srs = osr.SpatialReference()            # establish encoding
        srs.ImportFromEPSG(4326)                # WGS84 lat/long
        
        if len(image.shape) == 3:
            dst_ds = gdal.GetDriverByName('GTiff').Create(filename, nx, ny, 3, gdal.GDT_Byte)
            # dst_ds = gdal.GetDriverByName('GTiff').Create(filename, ny, nx, 3, gdal.GDT_Float64)
            dst_ds.SetGeoTransform(geotransform)    # specify coords
            srs = osr.SpatialReference()            # establish encoding
            srs.ImportFromEPSG(4326)                # WGS84 lat/long
            dst_ds.SetProjection(srs.ExportToWkt())  # export coords to file
            dst_ds.GetRasterBand(1).WriteArray(image[:, :, 0])   # write r-band to the raster
            dst_ds.GetRasterBand(2).WriteArray(image[:, :, 1])   # write g-band to the raster
            dst_ds.GetRasterBand(3).WriteArray(image[:, :, 2])   # write b-band to the raster 
        dst_ds.FlushCache()                     # write to disk
        dst_ds = None

        
if __name__ == '__main__':
    try:
        rospy.init_node('image_georeferencer')
        image_georeferencer = image_georeferencer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass