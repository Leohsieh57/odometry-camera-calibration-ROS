#!/usr/bin/env python3
import rospy
import numpy as np
from numpy.linalg import inv
import cv2
from nav_msgs.msg import Odometry
from gmapping_camera_calibration.srv import OdometryLists
import calibration_functions as cf

saveDir = rospy.get_param('odometry_camera_calibration/directory')

if __name__ == "__main__":
    rospy.init_node('calibration_result_visaulizer')

    #load results and data
    odomList = cf.loadData(saveDir+'laser_odom.npy')
    odomListCam = cf.loadData(saveDir+'camera_odom.npy')
    Tcr, Tbw  = cf.loadData(saveDir+'Tcr.npy')[0], cf.loadData(saveDir+'Tbw.npy')[0]

    #to Lie group
    Trc, Tbw = inv(cf.toSE3d(Tcr)), cf.toSE3d(Tbw)
    odomListCam = [cf.toSE3d(odom) for odom in odomListCam]

    #calculate calibrated camera poses
    TcwList = [np.dot(Tcb, Tbw) for Tcb in odomListCam]
    TrwList = [np.dot(Trc, Tcw) for Tcw in TcwList]
    TwrList = [inv(Trw) for Trw in TrwList]
    calibratedOdomList = [cf.toOdometryFromSE3d(Twr) for Twr in TwrList]

    #publish the odometries
    camPub = rospy.Publisher('calibrated_camera_pose', Odometry, queue_size=10)
    urgPub = rospy.Publisher('robot_pose', Odometry, queue_size=10)
    rate = rospy.Rate(30)

    #start while loop
    while not rospy.is_shutdown():
        for odomCam, odomLaser in zip(calibratedOdomList, odomList):
            camPub.publish(odomCam)
            urgPub.publish(odomLaser)
            rate.sleep()
    rospy.spin()