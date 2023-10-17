#!/usr/bin/env python

# For ROS
import rospy
from geometry_msgs.msg import PoseStamped

# For Images and Camera
import cv2 
import numpy as np
from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
from scipy.spatial.transform import Rotation
import math

# For Action Server
import actionlib
from eye.msg import VisionAction, VisionGoal, VisionResult
from constants import *
import json
import os


class Vision():

    # =========
    # Callbacks
    # =========
    def callback_actionRequest(self,msg):

        print("Request Recieved")
        
        goal_dict = json.loads(msg.request)

        self.actionHeader = goal_dict['header']
        self.actionData = goal_dict['data']
        self.action_requested_b = True
        self.actionSuccess = False
        self.actionFeedback = self.aruco_pose_str

        startTime = rospy.Time.now()

        while (rospy.Time.now() - startTime) < rospy.Duration(self.actionTimeout):
            if not self.action_requested_b:
                break
            rospy.sleep(0.1)

        if self.action_requested_b:
            print("timeout reached")
            self.action_requested_b = False

        feedback_dict = {'success':self.actionSuccess, 'feedback':self.actionFeedback}

        msg = VisionResult()
        msg.result = json.dumps(feedback_dict)

        self.actionSrv.set_succeeded(msg)

    def callback_camera(self,msg):
        try:
            if not self.queue_processImage:
                self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
                self.queue_processImage = True
        except CvBridgeError as e:
            print(e)
            
    def callback_cameraInfo(self,msg):
        # K = [ fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # D = [ k1, k2, p1, p2, k3]
        mtx = np.asarray(msg.K)
        self.cam_dist = np.asarray(msg.D)
        self.cam_mtx = np.zeros((3,3),dtype=np.float)
        self.cam_mtx[2,2] = 1
        self.cam_mtx[0,0] = mtx[0]
        self.cam_mtx[1,1] = mtx[4] 
        self.cam_mtx[0,2] = mtx[2] 
        self.cam_mtx[1,2] = mtx[5]

        # print ("MTX: {}".format(self.cam_mtx))
        # print ("DIST: {}".format(self.cam_dist))
        
        self.cameraInfoUpdate_b = True
        self.sub_cameraInfo.unregister()
        
        print("Vision Ready!")

    def showCrossHair(self):
        h,w = self.frame.shape[:2]
        

        frame = cv2.line(self.frame,(int(w/2),0),(int(w/2),h),(255,0,0),1)
        frame = cv2.line(self.frame,(0,int(h/2)),(w,int(h/2)),(255,0,0),1)


        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

        self.pub_img.publish(msg)
        
    def detectAruco(self):
        if not self.cameraInfoUpdate_b:
            return
    
        success = True
    
        if self.detectGray_b:
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)
            
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tostring()

            # self.pub_blur_img.publish(msg)

            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(self.frame, self.arucoDict, parameters=self.arucoParams)
    
        if isinstance(ids, type(None)):
            print("No fiducial detected!")
            success = False
        elif len(ids) == 0:
            print("No fiducial detected!")
            success = False
        if len(corners) > 0 and success:
            success = True
            if self.drawAxes_b:
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.cam_mtx,
                                                                                  self.cam_dist)
                    cv2.aruco.drawAxis(self.frame, self.cam_mtx, self.cam_dist, rvec, tvec, 0.02)
                    self.rvec_aruco = rvec[0][0]
                    self.tvec_aruco = tvec[0][0]
                    rospy.loginfo(self.tvec_aruco)
                    rospy.loginfo(type(self.rvec_aruco))
                    
        
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(self.frame, corners, ids)
            self.Aruco_pose()
    
        self.actionSuccess = success
        self.action_requested_b = False
    
        self.showCrossHair()
            
    def detectHole(self):

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
        closest_hole = None
        closest_dist = float('inf')
        target_radius = 20
        
        if circles is not None :
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                center = (circle[0], circle[1])
                radius = circle[2]
                dist_x = abs(center[0]  - self.frame.shape[1] // 2)
                dist_y = abs(center[1]  - self.frame.shape[0] // 2)

                dist = dist_x + dist_y
                if dist < closest_dist and abs(radius - target_radius) < target_radius:
                    closest_hole = circle 
                    closest_dist = dist
        if closest_hole is not None:
            center = (closest_hole[0], closest_hole[1])
            radius = closest_hole[2]
            cv2.circle(self.frame, center, radius, (0, 255, 0), 2)         

            
           
            
            tvec_aruco =  self.tvec_aruco  
            tvec_cam = np.array([0, 0, 0.1])
            try:
                self.tvec = tvec_aruco - tvec_cam
            except:
                rospy.loginfo(type(tvec_aruco))
                rospy.loginfo(type(tvec_cam))
            
        
        self.actionSuccess = True
        self.action_requested_b = False

        self.showCrossHair()        
    def saveImage(self):

        cwd  =  os.getcwd()
        targetPath = '/'.join(self.basePath.split('/')[:-1]) + "/media"

        if os.path.exists(targetPath):
            filename = self.actionData + ".jpg"
            os.chdir(targetPath)
            cv2.imwrite(filename,self.frame)

            print ("image saved: {}".format(filename))

        os.chdir(cwd)

        self.actionSuccess = True
        self.action_requested_b = False

        self.showCrossHair()
    def Hole_pose(self, position, orientation):
        hpose_msg = PoseStamped()
        hpose_msg.header.stamp = rospy.Time.now()
        hpose_msg.header.frame_id = "hole" 
        
        hpose_msg.pose.position.x = self.tvec[0]
        hpose_msg.pose.position.y = self.tvec[1]
        hpose_msg.pose.position.z = self.tvec[2]

        #quat = Rotation.from_dcm(self.rvec_aruco.as_quat())
        #pose_msg.orientation.x = quat[0]
        #pose_msg.orientation.y = quat[1]
        #pose_msg.orientation.z = quat[2]
        #pose_msg.orientation.w = quat[3]  
        
        self.pose_ho_publisher.publish(hpose_msg)
        
    def Aruco_pose(self):
        # pose_msg = PoseStamped()
        # pose_msg.header.stamp = rospy.Time.now()
        # pose_msg.header.frame_id = "aruco" 
        
        # pose_msg.pose.position.x = self.tvec_aruco[0]
        # pose_msg.pose.position.y = self.tvec_aruco[1]
        # pose_msg.pose.position.z = self.tvec_aruco[2]

        # quat = Rotation.from_euler('xyz', self.rvec_aruco).as_quat()
        # pose_msg.pose.orientation.x = quat[0]
        # pose_msg.pose.orientation.y = quat[1]
        # pose_msg.pose.orientation.z = quat[2]
        # pose_msg.pose.orientation.w = quat[3]  
        
        # self.pose_Aruco_publisher.publish(pose_msg)
        
        quat = Rotation.from_rotvec(self.rvec_aruco).as_quat()
        # # NOT A SOLUTION
        # euler = Rotation.from_quat(quat).as_euler('xyz')
        # euler[2] += 3.14159
        # quat = Rotation.from_euler('xyz', euler).as_quat()
        
        self.aruco_pose_str = str(self.tvec_aruco[0]) + ', ' + str(self.tvec_aruco[1]) + ', ' + str(self.tvec_aruco[2]) \
          + ', ' + str(quat[0]) + ', ' + str(quat[1]) + ', ' + str(quat[2]) + ', ' + str(quat[3])
         
        rospy.loginfo(self.aruco_pose_str)
        
    def __init__(self):

        self.basePath = "/home/ku-user/aric_ws/src/eye/scripts"
        self.pose_Aruco_publisher = rospy.Publisher('/camera_pose', PoseStamped, queue_size=1)
        self.pose_hole_publisher = rospy.Publisher('/hole_pose', PoseStamped, queue_size=1)

        # global image_publisher, image_subscriber
        rospy.init_node('vision')
        self.sub_img = rospy.Subscriber('/camera/color/image_raw',Image, self.callback_camera)
        self.pub_img = rospy.Publisher('/vision/image/compressed',CompressedImage, queue_size = 1)
        self.pub_blur_img = rospy.Publisher('/vision/image/blurred',Image, queue_size = 1)

        # Initialise Action Server
        self.action_requested_b = False
        self.actionHeader = Action.reset.name
        self.actionData = ""
        self.actionSrv = actionlib.SimpleActionServer('vision', VisionAction, self.callback_actionRequest,False)
        self.actionSrv.start()
        self.actionTimeout = 7.0
        
        
        self.bridge = CvBridge()

        self.rate = rospy.Rate(50)
        self.queue_processImage = False

        # Hole Detection Parameters
        self.min_radius = 5
        self.max_radius = 30
        self.threshold = 20
        self.tvec = None
        # Setting the Aruco Detector Parameters
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.arucoParams =  cv2.aruco.DetectorParameters_create()
        # self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        # self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.rvec_aruco = None
        self.tvec_aruco = None
        self.aruco_pose_str = ''
        # Flags
        self.detectGray_b = True
        self.drawAxes_b = True
        self.queue_processImage_b = False
        
        #TODO: Change this after camera Calibration on robot arm
        self.cameraInfoUpdate_b = False
        
        self.sub_cameraInfo = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.callback_cameraInfo,
                                               queue_size=1)

        while not rospy.is_shutdown():
            self.rate.sleep()
            
            if self.queue_processImage:
                if self.action_requested_b:
                    if self.actionHeader == Action.captureImage.name:
                        self.saveImage()
                    if self.actionHeader == Action.detectFiducial.name:
                        self.detectAruco()
                    if self.actionHeader == Action.detectHole.name:
                        self.detectHole()
                if not self.action_requested_b:
                    self.showCrossHair()

                self.queue_processImage = False
                    


if __name__ == '__main__':
    
    try:
       Vision()  
    except rospy.ROSInterruptException:
        pass
