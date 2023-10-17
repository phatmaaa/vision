#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import math
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge,CvBridgeError
from datetime import datetime

class HoleDetector():

    # =========
    # Callbacks
    # =========
    def callback_camera(self,msg):
        try:
            if not self.queue_processImage:
                self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
                self.queue_processImage = True
        except CvBridgeError as e:
            print(e)

    def detect_circles(self):
        # Convert frame to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Apply Hough Circle Transform
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                    param1=50, param2=self.threshold,
                                    minRadius=self.min_radius, maxRadius=self.max_radius)

        # Draw detected circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                center = (circle[0], circle[1])
                radius = circle[2]
                cv2.circle(self.frame, center, radius, (0, 255, 0), 2)

                #circle position relative to the camera
                #focal_length = 1
                #known_radius = 10
                #distance = (known_radius*focal_length)/radius
                #angle_x = math.degrees(math.atan((center[0]-self.frame.shape[1]/2)/focal_length))
                #angle_y = math.degrees(math.atan((center[1]-self.frame.shape[0]/2)/focal_length))
                #x = distance*math.sin(math.radians(angle_x))
                #y = distance*math.sin(math.radians(angle_y))
                #z = distance*math.cos(math.radians(angle_x))*math.cos(math.radians(angle_y))
                #self.circle_position = (x,y,z)
                #print ("circle position: {}".format(self.circle_position))

        # Display the frame
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

        self.pub_img.publish(msg)
            
        
        
    def __init__(self):
        # global image_publisher, image_subscriber
        rospy.init_node('hole_detector')
        self.sub_img = rospy.Subscriber('/camera/color/image_raw',Image, self.callback_camera)
        self.pub_img = rospy.Publisher('/hole_detector/image/compressed',CompressedImage, queue_size = 1)

        self.bridge = CvBridge()

        self.rate = rospy.Rate(50)
        self.queue_processImage = False

        # Hole Detection Parameters
        self.min_radius = 5
        self.max_radius = 30
        self.threshold = 20

        

        while not rospy.is_shutdown():
            self.rate.sleep()
            
            if self.queue_processImage:
                self.detect_circles()
                self.queue_processImage = False
                    




        rospy.spin()

if __name__ == '__main__':
    try:
       HoleDetector()  
    except rospy.ROSInterruptException:
        pass