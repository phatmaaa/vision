#!/usr/bin/env python
import cv2
import os
import glob
import numpy as np



cwd = os.getcwd()
target = '/'.join(cwd.split('/')[:-1])  +'/media'
baseName = "calibration_"
index = 0
fileType = '.jpg'

def captureImage():
    global cwd,target,baseName,index,fileType
    # Check if media directory Exists
    if not os.path.exists(target):
        os.mkdir(target)
    os.chdir(target)

    # Check is calibration images already exist
    available = glob.glob(target + '/' + baseName + "*" + fileType)
    if len(available) > index:  # update starting index
        index = int(sorted(available)[-1].split('.')[0].split('_')[-1]) + 1

    cam = cv2.VideoCapture(0)

    while True:
        ret, frame = cam.read()

        if ret:
            cv2.imshow('output', frame)
        key = cv2.waitKey(100)
        if key == 27:  # esc
            break
        if key == 99 or key == 13:  # c or enter
            fileName = baseName + str(index) + fileType
            index += 1
            cv2.imwrite(fileName, frame)
            print("saved as: {}".format(fileName))

    cam.release()
    # revert working directory
    os.chdir(cwd)

def tuneDetector():
    global cwd, target, baseName, index, fileType
    # Check if media directory Exists
    if not os.path.exists(target):
        raise  Exception("{} directory does not exist".format(target))
    os.chdir(target)

    # image = glob.glob(baseName+"*"+fileType)
    imageNames = glob.glob('calibrate_*.png')

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    rows = 7  # 7
    columns = 10  # 10
    objp = np.zeros((rows * columns, 3), np.float32)
    objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    objpoints = []
    imgpoints = []

    for name in imageNames:
        img = cv2.imread(name)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray, (3,3))
        chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        # ret, corners = cv2.findChessboardCorners(gray, (rows, columns), flags=chessboard_flags)  # chessboard_flags)
        # corners = cv2.cornerHarris(gray, 2, 3, 0.05)

        ret, corners = cv2.findChessboardCorners(gray, (3, 3), None)
        ret=True

        # print ("Operation: {}".format(ret))
        # rospy.sleep(0.2)
        if ret == True:
            # objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, (columns, rows), corners, ret)
            # img[corners > 0.01 * corners.max()] = [0, 0, 255]
            cv2.imshow('img', img)
            cv2.imshow('img_gray',gray)
            cv2.waitKey(500)
        # img = cv2.rectangle(img, (0, 0), (100, 100), (255, 0, 0), 1)





if __name__ == '__main__':

    captureImage()
    # tuneDetector()