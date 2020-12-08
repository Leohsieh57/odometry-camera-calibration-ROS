#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
import calibration_functions as cf

saveDir = rospy.get_param('odometry_camera_calibration/directory')

if __name__ == "__main__":
    rospy.init_node('corner_detector_offline')
    odomList = cf.loadData(saveDir+'laser_odom.npy')
    odomListCam = cf.loadData(saveDir+'camera_odom.npy')
    Tcr, Tbw = cf.callService(odomList, odomListCam, saveDir)
    rospy.spin()