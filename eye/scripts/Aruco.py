5

import rospy
import cv2
import json
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict
 
# bridge = CvBridge()

class Aruco():
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

        print ("MTX: {}".format(self.cam_mtx))
        print ("DIST: {}".format(self.cam_dist))
        
        self.cameraInfoUpdate_b = True
        
        self.sub_cameraInfo.unregister()
        print("Camera_Info Updated!")

    def process_frame(self):
        if not self.cameraInfoUpdate_b:
            return
        if self.detectGray:
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, self.arucoDict,parameters=self.arucoParams)
        else:
            (corners, ids, rejected) = cv2.aruco.detectMarkers(self.frame, self.arucoDict,parameters=self.arucoParams)
        
        if len(corners) > 0:
            if self.drawAxes:
                for i in range(0, len(ids)):
                    rvec, tvec, markerPoints =cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.cam_mtx, self.cam_dist)
                    cv2.aruco.drawAxis(self.frame, self.cam_mtx, self.cam_dist, rvec, tvec, 0.02)
                    print ("rvec: {}".format(rvec))
                    print ("tvec: {}".format(tvec))
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(self.frame, corners, ids)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

        self.pub_img.publish(msg)
            


    def __init__(self):

        node_name ='aruco_detector'
        
        rospy.init_node(node_name)
        
        self.sub_img = rospy.Subscriber('/camera/color/image_raw',Image, self.callback_camera)
        self.pub_img = rospy.Publisher('/aruco_detector/image/compressed',CompressedImage, queue_size = 1)
        
        self.bridge = CvBridge()

        self.rate = rospy.Rate(50)

        # Setting the Aruco Detector Parameters
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        # Flags
        self.detectGray = True
        self.drawAxes = True
        self.queue_processImage = False

        # =========================
        #   Load Camera Calibration Values
        #   http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        # =========================

        # try:
        #     with open("/home/hp/mrym_ws/src/eye/config/camera.json",'r') as f:
        #         cameraParams = json.load(f,object_pairs_hook=OrderedDict)

        #     intrinsics = cameraParams['intrinsic']
        #     self.cam_dist = (intrinsics['k1'],intrinsics['k2'],intrinsics['p1'],intrinsics['p2'],intrinsics['k3'])
        #     self.cam_mtx = np.zeros((3,3),dtype=np.float)
        #     self.cam_mtx[2,2] = 1
        #     self.cam_mtx[0,0] = intrinsics['fx']
        #     self.cam_mtx[1,1] = intrinsics['fy'] 
        #     self.cam_mtx[0,2] = intrinsics['cx'] 
        #     self.cam_mtx[1,2] = intrinsics['cy']
        # except:
        #     raise Exception("{}: Could not load from 'camera.json'".format(node_name))

        # ==> Get Camera Info from ROS
        #TODO: Change this after camera Calibration on robot arm
        self.cameraInfoUpdate_b = False
        
        self.sub_cameraInfo = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.callback_cameraInfo,
                                               queue_size=1)
    

        while not rospy.is_shutdown():
            self.rate.sleep()
            
            if self.queue_processImage:
                self.process_frame() 
                
                # msg = CompressedImage()
                # msg.header.stamp = rospy.Time.now()
                # msg.format = "jpeg"
                # msg.data = np.array(cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

                # self.pub_img.publish(msg)

                self.queue_processImage = False
            

if __name__ == '__main__':
    try:
        Aruco()
    except rospy.ROSInterruptException:
        pass
 
   
