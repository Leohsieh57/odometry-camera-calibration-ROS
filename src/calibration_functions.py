#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from copy import copy
from scipy.spatial.transform import Rotation as R
from odometry_camera_calibration.srv import OdometryLists
import yaml

def norm(newOdom, latestOdom):
    newPoint, oldPoint = newOdom.pose.pose.position, latestOdom.pose.pose.position
    displacement = np.array([newPoint.x-oldPoint.x,newPoint.y-oldPoint.y,newPoint.z-oldPoint.z])
    return np.linalg.norm(displacement)

def getObjPoints(nRows, nCols):
    objp = np.zeros((nRows*nCols ,3), np.float32)
    objp[:,:2] = np.mgrid[0:nRows,0:nCols].T.reshape(-1,2)
    return objp

def toList(odom):
    q = odom.pose.pose.orientation
    p = odom.pose.pose.position
    return [q.w, q.x, q.y, q.z, p.x, p.y, p.z]

def toOdometry(odomVec):
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.child_frame_id = "map"
    odom.pose.pose.orientation.w = odomVec[0]
    odom.pose.pose.orientation.x = odomVec[1]
    odom.pose.pose.orientation.y = odomVec[2]
    odom.pose.pose.orientation.z = odomVec[3]
    odom.pose.pose.position.x = odomVec[4]
    odom.pose.pose.position.y = odomVec[5]
    odom.pose.pose.position.z = odomVec[6]
    return odom

def toOdometryFromRt(rvec, tvec):
    odom = Odometry()
    q = R.from_rotvec(rvec[:,0]).as_quat().tolist()
    q = [q[-1]]+q[:3]
    return toOdometry(q+tvec[:,0].tolist())

def callService(odomList, odomListCam, saveDir):
    rospy.loginfo("service called")
    rospy.wait_for_service('calibration_server')
    try:
        getExtrinsicParams = rospy.ServiceProxy('calibration_server', OdometryLists)
        resp = getExtrinsicParams(odomListCam, odomList)
        Tcr, Tbw = [resp.Tcr], [resp.Tbw]
        saveData(saveDir+'Tcr.npy', Tcr)
        saveData(saveDir+'Tbw.npy', Tbw)
        return resp.Tcr, resp.Tbw

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def saveParams(camMat, dist, Tcr, saveDir):
    q = Tcr.pose.pose.orientation
    p = Tcr.pose.pose.position
    data = dict(
        intrinsicParams = dict(
            fx = str(camMat[0][0]),
            fy = str(camMat[1][1]),
            cx = str(camMat[0][2]),
            cy = str(camMat[1][2]),
            k1 = str(dist[0][0]),
            k2 = str(dist[0][1]),
            p1 = str(dist[0][2]),
            p2 = str(dist[0][3]),
            k3 = str(dist[0][4])
        ),
        extrinsicParams = dict(
            Tcr_qw = str(q.w),
            Tcr_qx = str(q.x),
            Tcr_qy = str(q.y),
            Tcr_qz = str(q.x),
            Tcr_px = str(p.x),
            Tcr_py = str(p.y),
            Tcr_pz = str(p.z)
        )
    )
    fn = saveDir+'calibration_results.yml'
    with open(fn, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)
    print('data successfully saved as',fn)

    

def saveData(fn, odomList):
    odomList = np.array([toList(odom) for odom in odomList])
    np.save(fn, odomList)
    print('data successfully saved at',fn)

def loadData(fn):
    odomList = np.load(fn) 
    print('data successfully loaded at',fn)
    return [toOdometry(odom) for odom in odomList]

def toSE3d(odom):
    q, p = [], []
    q.append(odom.pose.pose.orientation.x)
    q.append(odom.pose.pose.orientation.y)
    q.append(odom.pose.pose.orientation.z)
    q.append(odom.pose.pose.orientation.w)
    p.append(odom.pose.pose.position.x)
    p.append(odom.pose.pose.position.y)
    p.append(odom.pose.pose.position.z)

    rot = R.from_quat(q).as_matrix()
    trans = np.array([p]).T
    botRow = np.array([[0,0,0,1]])
    T = np.concatenate((rot, trans), axis=1)
    T = np.concatenate((T, botRow), axis=0)
    #print(T)
    return T

def toOdometryFromSE3d(T):
    q = R.from_matrix(T[0:3,0:3]).as_quat().tolist()
    q = [q[-1]]+q[:3]
    tvec = T[0:3,3:4]
    return toOdometry(q+tvec[:,0].tolist())

