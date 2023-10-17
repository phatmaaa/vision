#!/usr/bin/env python
import rospy
import cv2
import glob
import json
from sensor_msgs.msg import CompressedImage#, Image
# from cv_bridge import CvBridge
from datetime import datetime
import numpy as np
import os

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
rows = 7#7
columns = 10#10
objp = np.zeros((rows * columns, 3), np.float32)
objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
objpoints = []
imgpoints = []
# bridge = CvBridge()

img = np.zeros(1)

def image_callback(msg):
    global objpoints, imgpoints, image_publisher,img
    np_arr = np.fromstring(msg.data,np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    min_radius = 5
    max_radius = 30
    threshold = 20

    detect_circles(min_radius, max_radius, threshold)
    #publishImage(img)

def detect_circles(min_radius, max_radius, threshold):

    global img

    frame = img.copy()

    # Initialize the camera
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(gray_blur, 50, 150)

    # Apply Hough Circle Transform
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                               param1=50, param2=threshold,
                               minRadius=min_radius, maxRadius=max_radius)

    # Draw detected circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]
            cv2.circle(frame, center, radius, (0, 255, 0), 2)

    # Display the frame
    # cv2.imshow('Circles Detection', frame)
    publishImage(frame)
    # Exit if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    # Release the camera and close the windows
    # cap.release()
    # cv2.destroyAllWindows()

# Get user input for minimum radius, maximum radius, and threshold


# Call the detect_circles function with user inputs


def publishImage(img):
    global image_publisher
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])[1]).tostring()

    image_publisher.publish(msg)

def process_image():
    global img
    global objpoints, imgpoints
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray,(3,3))
    chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, (rows, columns), flags=chessboard_flags) #chessboard_flags)

    # print ("Operation: {}".format(ret))
    # rospy.sleep(0.2)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, (columns, rows), corners2, ret)
        publishImage(img)
        rospy.sleep(1.0)
        # cv.imshow('img', img)
        # cv.waitKey(1500)
    img = cv2.rectangle(img,(0,0),(100,100),(255,0,0),1)
    publishImage(img)
    rospy.sleep(0.2)


def calibrate_camera():
    global img, image_publisher

    baseDir = '/home/hp/mrym_ws/src/eye/scripts'

    rospy.init_node('calibration')
    image_publisher = rospy.Publisher("/calibrated_node/image/compressed", CompressedImage, queue_size=1)

    cwd = os.getcwd()
    os.chdir(baseDir)
    wd = os.getcwd()
    target = '/'.join(wd.split('/')[:-1]) + '/media'
    if os.path.exists(target):
        os.chdir(target)

    print(os.getcwd())

    images = glob.glob('calibrate_*.png')
    print(len(images), "images found")
    imgShape = None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray,(3,3))
        chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, (rows, columns), flags=chessboard_flags)
        # print("+++++ fname")
        # print(len(corners))

        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, (columns, rows), corners2, ret)

            publishImage(img)
            rospy.sleep(0.05)

        if isinstance(imgShape, type(None)):
            imgShape= gray.shape[::-1]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, imgShape, None, None)
    camera = {}
    camera['intrinsic'] = {'fx': mtx[0, 0],
                          'fy': mtx[1, 1],
                          'cx': mtx[0, 2],
                          'cy': mtx[1, 2],
                          'k1': dist[0, 0],
                          'k2': dist[0, 1],
                          'p1': dist[0, 2],
                          'p2': dist[0, 3],
                          'k3': dist[0, 4],
                          'w': gray.shape[0],
                          'h': gray.shape[1]}

    for i in np.arange(0, len(tvecs)):
        H = np.zeros([4, 4])
        H[0:3, 3] = np.dot(tvecs[i].ravel(), 1e-3)
        H[0:3, 0:3] = cv2.Rodrigues(rvecs[i])[0]
        H[3, 3] = 1

        camera['pose-{:03d}'.format(i)]={'board_wrt_camera' : H.tolist()}

    # change to config directory
    target = '/'.join(wd.split('/')[:-1]) + '/config'

    if not os.path.exists(target):
        os.mkdir(target)

    os.chdir(target)

    with open("camera.json", 'w') as f:
        json.dump(camera, f, indent=4)

    print("camera calibration saved")

    # revert working directory
    os.chdir(cwd)


def main():
    global image_publisher, image_subscriber
    rospy.init_node('calibration_node', anonymous=True)
    image_subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, image_callback)
    image_publisher = rospy.Publisher("/calibrated_node/image/compressed", CompressedImage, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        # main()
        calibrate_camera()
    except rospy.ROSInterruptException:
        pass
