#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from copy import copy
import calibration_functions as cf

bridge = CvBridge()
latestOdom = Odometry()
latestImgBgr8, latestImgMono = None, None
serverCalled, imgRecieved = False, False

nRows = rospy.get_param('odometry_camera_calibration/checkerBoardParam/nRows')
nCols = rospy.get_param('odometry_camera_calibration/checkerBoardParam/nCols')
cellSize = rospy.get_param('odometry_camera_calibration/checkerBoardParam/cellSize')
nFrames = rospy.get_param('odometry_camera_calibration/checkerBoardParam/nFrames')
saveDir = rospy.get_param('odometry_camera_calibration/directory')
paramDir = rospy.get_param('odometry_camera_calibration/paramDirectory')

objPoints, imgPoints, odomList = [], [], []

def robotMoved(newOdom):
    global latestOdom
    odomChanged = cf.norm(newOdom, latestOdom) > 0.04
    latestOdom = copy(newOdom) if odomChanged else latestOdom
    return odomChanged

def computeCamPose():
    rospy.loginfo("waiting for camera pose optimization...")
    rospy.loginfo("this may take a while based on the number of sampled pictures")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, latestImgMono.shape, None, None)
    odomListCam = [cf.toOdometryFromRt(rvecs[i], tvecs[i]) for i in range(nFrames)]
    cf.saveData(saveDir+'laser_odom.npy', odomList)
    cf.saveData(saveDir+'camera_odom.npy', odomListCam)
    Tcr, Tbw = cf.callService(odomList, odomListCam, saveDir)
    cf.saveParams(mtx, dist, Tcr, paramDir)

def poseCallBack(msg):
    global serverCalled
    global imgRecieved
    global latestImgMono
    global latestImgBgr8
    imgBgr8, imgMono = copy(latestImgBgr8), copy(latestImgMono)
    if imgRecieved and robotMoved(msg):
        if len(objPoints) < nFrames:
            rospy.loginfo("currently %d frames", len(odomList))
            cornerCaptured, corners = cv2.findChessboardCorners(imgMono, (nRows, nCols), None)
            if cornerCaptured:
                objPoints.append(cellSize*cf.getObjPoints(nRows, nCols))
                imgPoints.append(corners)
                odomList.append(msg)
                cv2.drawChessboardCorners(imgBgr8, (nRows, nCols), np.rint(corners), True)

        elif not serverCalled:
            serverCalled = True
            computeCamPose()

    if imgRecieved:
        cv2.imshow('detected points', imgBgr8)
        cv2.waitKey(1)

def imgCallBack(msg):
    global latestImgBgr8
    global latestImgMono
    global imgRecieved
    latestImgBgr8 = bridge.imgmsg_to_cv2(msg, "bgr8")
    latestImgMono = cv2.cvtColor(latestImgBgr8, cv2.COLOR_BGR2GRAY)
    imgRecieved = True


if __name__ == "__main__":
    rospy.init_node('corner_detector')
    rospy.Subscriber("camera_image", Image, imgCallBack, queue_size=1)
    rospy.Subscriber("robot_pose", Odometry, poseCallBack, queue_size=1)
    rospy.spin()