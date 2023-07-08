#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('cam_tune', anonymous = True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

Width = 640
Height = 480

calibrated = True

if calibrated:
    mtx = np.array([
        [374.313954, 0.000000, 313.884148],
        [0.000000, 375.505020, 257.925602],
        [0.000000, 0.000000, 1.000000]
    ])

    dist = np.array([-0.315565, 0.076723, 0.001097, 0.002779, 0.000000])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

i = 0

while not rospy.is_shutdown():

    if cv_image.size != (640*480*3):
        continue
    
    calibrated = calibrate_image(cv_image)

    # cv2.imshow("original", cv_image)
    cv2.imshow("calibrated", calibrated)
    padded_num = str(i).zfill(5)
    name = '/home/nvidia/output_image/' + padded_num + '.png'
    cv2.imwrite(name, calibrated)
    i += 1
    print(name)
    cv2.waitKey(1000)