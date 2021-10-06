#!/usr/bin/env python2

import pygame
import sys
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
import textwrap 
import math
import sensor_msgs.point_cloud2 as pc2
from obj_segmentation.msg import SegmentedClustersArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from image_geometry import PinholeCameraModel
from husky_moveit_config.srv import Indicate

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.8
FONT_COLOR = (255,255,255)
FONT_THICKNESS = 1
LINE_TYPE = 2

def get_button_image(img, text):
	wrapped_text = textwrap.wrap(text, width=35)
	x, y = 5, 10
	font_size = 1
	font_thickness = 2

	for i, line in enumerate(wrapped_text):
	    textsize = cv2.getTextSize(line, FONT, FONT_SCALE, FONT_THICKNESS)[0]
	    gap = textsize[1] + 10
	    y = int((img.shape[0] + textsize[1]) / 2) + i * gap
	    x = 0
	    cv2.putText(img, line, (x, y), FONT, FONT_SCALE, FONT_COLOR, LINE_TYPE)

	return img

class TestControl:    
    def __init__(self):
	pygame.init()
	self.pgscreen=pygame.display.set_mode((1080, 720))
	self.pgscreen.fill((255, 255, 255))
	pygame.display.set_caption('TestControl')
	rospy.init_node('TestControl')
        self.cam_info = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)
        #self.objects_sub = rospy.Subscriber('/object_clusters', SegmentedClustersArray, self.get_location)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.get_image)

        self.button_pub = rospy.Publisher('/buttons', String, queue_size=10)
        self.serv = rospy.ServiceProxy('indicate', Indicate)

        self.locations = []
        self.cvbridge = CvBridge()

	self.buttons = pygame.sprite.Group()
	self.names = {
		"What can you tell me about this thing?" : 0, 
		"Can you please describe this object?" : 1, 
		"What does it look like?" : 2, 
		"What is it used for?" : 3, 
		"Can you tell me more about this?" : 4, 
		"Is this hard or soft?" : 5, 
		"What shape is this?" : 6, 
		"What texture does it have?" : 7, 
		"What color is it?" : 8, 
		"Thank you!" : 9, 
		"Would you eat this for lunch?" : 10, 
		"Can you tell me why?" : 11
	}
	y = 60
	x = 860
	place = 20
	for name in self.names:
	    button = pygame.sprite.Sprite()
	    button.name = name
	    blank_image = get_button_image(np.zeros((30,360,3), np.uint8), name)
	    button.image = pygame.image.frombuffer(blank_image.tostring(), blank_image.shape[1::-1], "RGB")
	    button.rect = button.image.get_rect()
	    button.rect.center = (x, place)
	    place += 40
	    self.buttons.add(button)
	self.buttons.draw(self.pgscreen)

	while True:
		pygame.display.update()
		for event in pygame.event.get():
			#print(event)
			if event.type == pygame.MOUSEBUTTONDOWN:
				self.get_mouse()
			elif event.type == pygame.KEYDOWN:
				keys = pygame.key.get_pressed()
				print(keys)				
			elif event.type == pygame.QUIT:
				sys.exit()
	rospy.spin()

    def get_mouse(self):
	pos = pygame.mouse.get_pos()
	for i, button in enumerate(self.buttons):
		if button.rect.left < pos[0] < button.rect.right and button.rect.top < pos[1] < button.rect.bottom:
			print(button.name, self.names[button.name])
			self.button_pub.publish("play"+str(self.names[button.name]))

    def get_location(self, clusters):
	#print(len(clusters.clusters))
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
		break

            x = sum_x#/num_points
            y = sum_y#/num_points
            z = sum_z#/num_points
            self.locations.append([x,y,z])

    def get_image(self, img):
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        i = 0
        for location in self.locations:
            (u, v) = self.cam_model.project3dToPixel( location )
            cv2.putText(cv_image, str(i), (int(u),int(v)), FONT, FONT_SCALE, FONT_COLOR, LINE_TYPE)
            i += 1

	pg_img = pygame.image.frombuffer(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB).tostring(), cv_image.shape[1::-1], "RGB")
	self.pgscreen.blit(pg_img, (5,5))
	pygame.display.update()

if __name__=='__main__':
	TestControl()



