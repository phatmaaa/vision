#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
import time


class CameraNode():

    def pub_image(self):

        # img = np.zeros((640,320),dtype=np.float32)
        # img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        # img = cv2.rectangle(img,(10,10),(100,100),(255,0,0),-1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.img, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

        self.img_pub.publish(msg)

    def detect_features(self):
        method = "GridFAST"
        feat_det = cv2.FastFeatureDetector_create()
        startTime = rospy.Time.now()

        featPoints = feat_det.detect(cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY))
        endTime = rospy.Time.now()

        print("{} detector found: {} points in {} sec".format(method, len(featPoints), (endTime - startTime).to_sec()))

        for featPoint in featPoints:
            x, y = featPoint.pt
            cv2.circle(self.img, (int(x), int(y)), 3, (0, 0, 255), 1)

    def __init__(self):

        rospy.init_node("camera")

        self.rate = rospy.Rate(50)

        self.img_pub = rospy.Publisher("/camera/Image/compressed", CompressedImage, queue_size=1)

        self.cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            self.rate.sleep()

            try:
                ret, self.img = self.cap.read()
                if not ret:
                    continue
            except:
                pass

            # self.detect_features()

            self.pub_image()

        self.cap.release()


if __name__ == "__main__":
    CameraNode()