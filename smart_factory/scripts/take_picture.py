#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
from pypylon import pylon 


def take_picutre(file_path):
    """
    Basler相机拍照
    """
    num_img_to_save = 1
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

            filename = file_path
            img.Save(pylon.ImageFileFormat_Png, filename)

            # In order to make it possible to reuse the grab result for grabbing
            # again, we have to release the image (effectively emptying the
            # image object).
            img.Release()

    cam.StopGrabbing()
    cam.Close()

    return 0


def undistort_pic(dis_file_path, undis_file_path):
    """
    图像校正
    """
    with np.load('/home/pilz/Pictures/camera_calibration/test_fname.npz') as X:
        mtx = X['mtx']
        dist = X['dist']

    img = cv2.imread(dis_file_path)
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # 校正
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # 图像裁剪并保存
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite(undis_file_path, dst)
    return 0


if __name__ == "__main__":
    original_file_path = "/home/pilz/Pictures/smart_factory/cap.png" 
    calibrated_file_path = "/home/pilz/Pictures/smart_factory/cap_calibrated.png"
    take_picutre(original_file_path)
    undistort_pic(original_file_path, calibrated_file_path)
