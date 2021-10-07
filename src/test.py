#!/usr/bin/env python2

import pygame
import sys
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
import textwrap 
import time
import sensor_msgs.point_cloud2 as pc2
from obj_segmentation.msg import SegmentedClustersArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped
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

    for i, line in enumerate(wrapped_text):
        textsize = cv2.getTextSize(line, FONT, FONT_SCALE, FONT_THICKNESS)[0]
        gap = textsize[1] + 10
        y = int((img.shape[0] + textsize[1]) / 2) + i * gap
        x = 0
        cv2.putText(img, line, (x, y), FONT, FONT_SCALE, FONT_COLOR, LINE_TYPE)

    return img

class TestControl:    
    def __init__(self):
        rospy.init_node('TestControl')
        self.cam_info = rospy.wait_for_message('/camera/unityrgb/camera_info', CameraInfo)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(self.cam_info)
        self.objects_sub = rospy.Subscriber('/object_clusters', SegmentedClustersArray, self.get_location)
        self.image_sub = rospy.Subscriber('/camera/unityrgb/image_raw', Image, self.image_cb)
        self.robot_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.button_pub = rospy.Publisher('/buttons', String, queue_size=10)
        self.serv = rospy.ServiceProxy('indicate', Indicate)
        self.locations = []
        self.cvbridge = CvBridge()
        #rospy.spin()

    def has_image(self):
        has_image = False
        if self.cv_image.any():
            has_image = True
        return has_image

    def get_location(self, clusters):
        #print(clusters.header)
        #print(len(clusters.clusters))
        #del self.locations[:]
        loc = []
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
            loc.append([x,y,z])
            #print(self.locations)
        del self.locations[:]
        self.locations = loc

    def image_cb(self, img):
        #print(img.header)
        cv_image = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        
        i = 0
        for location in self.locations:
            (u, v) = self.cam_model.project3dToPixel( location )
            cv2.putText(cv_image, str(i), (int(u),int(v)), FONT, FONT_SCALE, FONT_COLOR, LINE_TYPE)
            i += 1
        self.cv_image = cv_image


if __name__=='__main__':
    tc = TestControl()
    time.sleep(2)
    pygame.init()
    pgscreen=pygame.display.set_mode((1080, 720))
    pgscreen.fill((255, 255, 255))
    pygame.display.set_caption('TestControl')

    buttons = pygame.sprite.Group()
    names = {
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
    for name in names:
        button = pygame.sprite.Sprite()
        button.name = name
        blank_image = get_button_image(np.zeros((30,360,3), np.uint8), name)
        button.image = pygame.image.frombuffer(blank_image.tostring(), blank_image.shape[1::-1], "RGB")
        button.rect = button.image.get_rect()
        button.rect.center = (x, place)
        place += 40
        buttons.add(button)
    buttons.draw(pgscreen)
    pygame.display.update()
    
    while True:
        if tc.has_image():
            pg_img = pygame.image.frombuffer(cv2.cvtColor(tc.cv_image, cv2.COLOR_BGR2RGB).tostring(), tc.cv_image.shape[1::-1], "RGB")
            pgscreen.blit(pg_img, (5,5))
            pygame.display.update()
        
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                for i, button in enumerate(buttons):
                    if button.rect.left < pos[0] < button.rect.right and button.rect.top < pos[1] < button.rect.bottom:
                        print(button.name, names[button.name])
                        tc.button_pub.publish("play"+str(names[button.name]))
            elif (event.type == pygame.KEYDOWN):
                keys = pygame.key.get_pressed()
                if keys[pygame.K_UP] or keys[pygame.K_DOWN] or keys[pygame.K_RIGHT] or keys[pygame.K_LEFT]:
                    cmd = Twist()
                    if keys[pygame.K_UP]:
                        print('K_UP arrow key')
                        cmd.linear.x = 1.0
                    elif keys[pygame.K_DOWN]:
                        print('K_DOWN arrow key')
                        cmd.linear.x = -1.0
                    elif  keys[pygame.K_RIGHT]:
                        print('K_RIGHT arrow key')
                        cmd.angular.z = -1.0
                    elif  keys[pygame.K_LEFT]:
                        print('K_LEFT arrow key')
                        cmd.angular.z = 1.0
                    print(cmd)
                    tc.robot_cmd_vel.publish(cmd)
                if (keys[pygame.K_0] or keys[pygame.K_1] or keys[pygame.K_2] or keys[pygame.K_3] or
                    keys[pygame.K_4] or keys[pygame.K_5] or keys[pygame.K_6] or keys[pygame.K_7] or
                    keys[pygame.K_8] or keys[pygame.K_9]):
                    key = -1
                    if keys[pygame.K_0]:
                        print('K_0 key')
                        key = 0
                    elif keys[pygame.K_1]:
                        print('K_1 key')
                        key = 1
                    elif  keys[pygame.K_2]:
                        print('K_2 key')
                        key = 2
                    elif  keys[pygame.K_3]:
                        print('K_3 key')
                        key = 3
                    elif keys[pygame.K_4]:
                        print('K_4 key')
                        key = 4
                    elif  keys[pygame.K_5]:
                        print('K_5 key')
                        key = 5
                    elif  keys[pygame.K_6]:
                        print('K_6 key')
                        key = 6
                    elif keys[pygame.K_7]:
                        print('K_7 key')
                        key = 7
                    elif  keys[pygame.K_8]:
                        print('K_8 key')
                        key = 8
                    elif  keys[pygame.K_9]:
                        print('K_9 key')
                        key = 9
                    p = PointStamped()
                    print(key, len(tc.locations))
                    if (0<= key < len(tc.locations)):
                        p.header.frame_id = 'kinect2_link'
                        p.point.x = tc.locations[key][0]
                        p.point.y = tc.locations[key][1]-0.15
                        p.point.z = tc.locations[key][2]
                        print(p)
                        try:
                            resp = tc.serv(p)
                            print(resp)
                        except:
			                pass
                if (keys[pygame.K_s]):
                    print('K_s key')
                    tc.button_pub.publish("home")
                    
            elif event.type == pygame.QUIT:
                sys.exit()
    
