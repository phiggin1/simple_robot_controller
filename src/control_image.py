#! /usr/bin/env python2

import rospy
import sys
import numpy as np
import cv2
import math
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from obj_segmentation.msg import SegmentedClustersArray
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

class control_image:    
    def __init__(self):
        rospy.init_node('control_image', anonymous=True)
        self.cam_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)
        self.objects_sub = rospy.Subscriber('/object_clusters', SegmentedClustersArray, self.get_location)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_image)
        self.control_image_pub = rospy.Publisher('/control_image', Image, queue_size=10)
        self.locations = []
        self.cvbridge = CvBridge()
        rospy.spin()

    def get_location(self, clusters):
        del self.locations[:]
        for i, pc in enumerate(clusters.clusters):
            num_points = pc.width * pc.height
            #print("Cluster", i ,": # points ", num_points)
            sum_x = 0
            sum_y = 0
            sum_z = 0
            #for all points in the cluster i
            for p in pc2.read_points(pc):
                sum_x = sum_x + p[0]
                sum_y = sum_y + p[1]
                sum_z = sum_z + p[2]

            x = sum_x/num_points
            y = sum_y/num_points
            z = sum_z/num_points
            self.locations.append([x,y,z])
        print(self.locations)

    def get_image(self, img):
        print(len(self.locations))
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        #cv_image = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2

        i = 0
        for location in self.locations:
            (u, v) = self.cam_model.project3dToPixel( location )
            cv2.putText(cv_image, str(i), (int(u),int(v)), font, fontScale, fontColor, lineType)
            i += 1

        image_message = self.cvbridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.control_image_pub.publish(image_message)



if __name__ == '__main__':
    c = control_image()

