#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob
from pypylon import pylon


def take_calibration_pic():
    """
    Basler摄像头拍棋盘格
    """
    num_img_to_save = 200
    img = pylon.PylonImage()
    tlf = pylon.TlFactory.GetInstance()

    cam = pylon.InstantCamera(tlf.CreateFirstDevice())
    cam.Open()
    cam.StartGrabbing()
    for i in range(num_img_to_save):
        with cam.RetrieveResult(2000) as result:

            # Calling AttachGrabResultBuffer creates another reference to the
            # grab result buffer. This prevents the buffer's reuse for grabbing.
            img.AttachGrabResultBuffer(result)

            filename = "/home/pilz/Pictures/camera_calibration/calibration_pic_%d.png" % i
            img.Save(pylon.ImageFileFormat_Png, filename)

            # In order to make it possible to reuse the grab result for grabbing
            # again, we have to release the image (effectively emptying the
            # image object).
            img.Release()

    cam.StopGrabbing()
    cam.Close()
    return 0


def execute_calibration():
    """
    执行标定程序，该段内容源于Opencv教程
    """
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((8*6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob('/home/pilz/Pictures/camera_calibration/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (6, 8), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            # img = cv2.drawChessboardCorners(img, (6, 8), corners2, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(500)

    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 存文件
    np.savez('/home/pilz/Pictures/camera_calibration/test_fname.npz', mtx = mtx, dist = dist)

    return (mtx, dist)


if __name__ == "__main__":
    take_calibration_pic()
    mtx, dist = execute_calibration()
    print(mtx)
    print(dist)
