#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np



def get_pen_pose(calibrated_file_path):
    pen_pickY_list = []
    original_image = cv2.imread(calibrated_file_path)
    pen_plate_roi = original_image[70:720, 70:620]
    image_copy = pen_plate_roi.copy()
    image_gray = cv2.cvtColor(pen_plate_roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(image_gray, 150, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Select inner contours
    try:
        hierarchy = hierarchy[0]
        for component in zip(contours, hierarchy):
            currentContour = component[0]
            currentHierarchy = component[1]
            if currentHierarchy[3] == -1:
                # these are the outermost child components
                img_contour = cv2.drawContours(image_copy, currentContour, -1, (0, 0, 0))
                if 5000 < cv2.moments(currentContour)['m00'] < 15000:
                    m00 = cv2.moments(currentContour)['m00']
                    m01 = cv2.moments(currentContour)['m01']
                    m10 = cv2.moments(currentContour)['m10']
                    centerX = round(m10/m00)
                    centerY = round(m01/m00)
                    x, y, w, h = cv2.boundingRect(currentContour)
                    points = np.array([[[x, y]], [[x+w, y]], [[x+w, y+h]], [[x, y+h]]])
                    cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                    pickY = (14 - centerY * 0.2687) / 1000
                    pen_pickY_list.append(pickY)
                    cv2.circle(img_contour, (int(centerX), int(centerY)), 5, (0, 0, 255))
        # cv2.imshow("img_contour", image_copy)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        pen_pickY_list = sorted(pen_pickY_list)
    except:
        pass
    return pen_pickY_list


if __name__ == '__main__':
    calibrated_file_path = '/home/pilz/Pictures/smart_factory/cap_calibrated.png'
    pen_y = get_pen_pose(calibrated_file_path)
    print(pen_y)
