#ifndef SLAM_MATHEMATICS_H
#define SLAM_MATHEMATICS_H

#include <vector>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>

using namespace Eigen;
using namespace Sophus;
using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 2, 6> Matrix26;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace SlamMath{
    class CameraParams{
        public:
            double fx, fy, cx, cy;
            int nRows, nCols;
            CameraParams(){}
            CameraParams(double _fx, double _fy, double _cx, double _cy){
                fx = _fx; fy = _fy; cx = _cx; cy = _cy;
            }
            CameraParams(double _fx, double _fy, double _cx, double _cy, int _nRows, int _nCols){
                fx = _fx; fy = _fy; cx = _cx; cy = _cy; nRows = _nRows; nCols = _nCols;
            }
            int GetCornerCnt(){return nRows*nCols;}
            Matrix3d GetCameraMatrix(){
                Matrix3d cameraMatrix;
                cameraMatrix << fx, 0, cx,
                                0, fy, cy,
                                0,  0,  1;
                return cameraMatrix;
            }
            Vector2d CameraProj(Vector3d p){return (GetCameraMatrix()*p).head(2)/p(2,0);}
            Vector2d CameraProj(Vector4d _p){
                Vector3d p = _p.head(3);
                return CameraProj(p);
            }
            Matrix26 JaccobianSE3ToProj(Vector3d p){
                Matrix26 jaccobian; 
                double x = p(0,0); double y = p(1,0); double z = p(2,0);
                double x_2 = x*x; double y_2 = y*y; double z_2 = z*z;
                jaccobian << fx/z, 0, -fx*x/z_2, -fx*x*y/z_2, fx+fx*x_2/z_2, -fx*y/z,
                             0, fy/z, -fy*y/z_2, -fy-fy*y_2/z_2, fy*x*y/z_2,  fy*x/z;
                return jaccobian;
            }
            Matrix26 JaccobianSE3ToProj(Vector4d _p){
                Vector3d p = _p.head(3);
                return JaccobianSE3ToProj(p);
            }       
    };
    Matrix6d Jr_inv(Vector6d eta){
        Matrix6d Jr;
        Sophus::SE3d e = Sophus::SE3d::exp(eta);
        Jr.block(0, 0, 3, 3) = Sophus::SO3d::hat(e.so3().log());
        Jr.block(0, 3, 3, 3) = Sophus::SO3d::hat(e.translation());
        Jr.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
        Jr.block(3, 3, 3, 3) = Sophus::SO3d::hat(e.so3().log());
        return Matrix6d::Identity() + 0.5*Jr + Jr*Jr/12.0;
    }
    Vector4d toVector4d(Vector3d _p){
        Vector4d p;
        p << _p, 1;
        return p;
    }
    Vector3d toVector3d(Vector4d _p){
        Vector3d p = _p.head(3);
        return p;
    }
    Sophus::SE3d toSE3d(nav_msgs::Odometry odom){
        geometry_msgs::Quaternion q = odom.pose.pose.orientation;
        geometry_msgs::Point p = odom.pose.pose.position;
        return Sophus::SE3d(Quaterniond(q.w, q.x, q.y, q.z), Vector3d(p.x, p.y, p.z));
    }
    vector<Sophus::SE3d> toSE3d(vector<nav_msgs::Odometry> odom){
        vector<Sophus::SE3d> result(odom.size());
        for (int i = 0; i < odom.size(); i++){
            result[i] = toSE3d(odom[i]);
        }
        return result;
    }
    nav_msgs::Odometry toOdometry(Sophus::SE3d T){
        Vector3d so3 = T.log().block<3,1>(3,0);
        Matrix3d R = Sophus::SO3d::exp(so3).matrix();
        Quaterniond q(R);
        Vector3d t = T.matrix().block<3,1>(0,3);
        nav_msgs::Odometry odom;
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.position.x = t(0,0);
        odom.pose.pose.position.y = t(1,0);
        odom.pose.pose.position.z = t(2,0);
        return odom;
    }
}
#endif
