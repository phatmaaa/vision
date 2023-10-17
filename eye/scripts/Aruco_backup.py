#!/usr/bin/env python

import rospy
import cv2 as cv
#from imutils.video import VideoStream
import argparse
import time
import json
import numpy as np
from sensor_msgs.msg import Image
#from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import sys
from collections import OrderedDict
 

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
                default="DICT_ARUCO_ORIGINAL",
                help="type of ArUCo tag to detect")
args = vars(ap.parse_args())
ARUCO_DICT = {
    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,

    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}
detectgray = True
drawaxes = True

if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
    sys.exit(0)
arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv.aruco.DetectorParameters()
bridge = CvBridge()

# =========================
#   Load Camera Calibration Values
# =========================

with open("/home/hp/mrym_ws/src/eye/config/camera.json",'r') as f:
    cameraParams = json.load(f,object_pairs_hook=OrderedDict)

intrinsics = cameraParams['intrinsic']
dist = (intrinsics['k1'],intrinsics['k2'],intrinsics['p1'],intrinsics['p2'],intrinsics['k3'])
mtx = np.zeros((3,3),dtype=np.float)
mtx[2,2] = 1
mtx[0,0] = intrinsics['fx']
mtx[1,1] = intrinsics['fy'] 
mtx[0,2] = intrinsics['cx'] 
mtx[1,2] = intrinsics['cy']
class Aruco():
    def process_frame(frame):
        global arucoDict, arucoParams, detectgray, drawaxes
        
        cap = cv.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if detectgray:
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                (corners, ids, rejected) = cv.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams)
            else:
                (corners, ids, rejected) = cv.aruco.detectMarkers(frame, arucoDict,parameters=arucoParams)
                if len(corners) > 0:
                    if drawaxes:
                        for i in range(0, len(ids)):
                            rvec, tvec, markerPoints =cv.aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                            cv.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.02)
                    ids = ids.flatten()
                    cv.aruco.drawDetectedMarkers(frame, corners, ids)
                cv.imshow("Frame", frame)
                cv.waitKey(1)
        cap.release()


    def image_callback(msg):
        try:
            #np_arr = np.fromstring(msg.data, np.unit8)
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
            #process_frame(frame)
        except CvBridgeError as e:
            print(e)


    def main():
        rospy.init_node( 'aruco_detection_node', anonymous=True)
        rospy.loginfo("Detecting '{}'; tags...".format(args["type"]))
        pub = rospy.Publisher('/camera/color/image_raw',Image, queque_size = 1)
    #rospy.Subscriber("/camera/color/image_raw",Image, image_callback)
        rospy.spin()
    #vs = VideoStream(src=0).start()
    #cap = cv.VideoCapture(0)
    #time.sleep(2.0)

    #while not rospy.is_shutdown():
        #frame = vs.read()
        
        #process_frame(frame)

    #cv.destroyAllWindows()
    #vs.stop()
    #cap.release()

if __name__ == '__main__':
    try:
        Aruco()
    except rospy.ROSInterruptException:
        pass
 
   
