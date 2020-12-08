#ifndef CALIBRATION_SERVER_H
#define CALIBRATION_SERVER_H

#include <ros/ros.h>
#include <vector>
#include <odometry_camera_calibration/OdometryLists.h>
#include <base_vertex.h>
#include <base_binary_edge.h>
#include <block_solver.h>
#include <optimization_algorithm_levenberg.h>
#include <linear_solver.h>
#include <linear_solver_dense.h>
#include <robust_kernel.h>
#include <robust_kernel_impl.h>
#include <SlamMathematics.h>
#include <assert.h>

using namespace Eigen;
using namespace Sophus;
using namespace std;

typedef vector<Eigen::Vector2d> cornerCoordinates;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType; 
typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
typedef odometry_camera_calibration::OdometryLists::Request ServiceRequestType;
typedef odometry_camera_calibration::OdometryLists::Response ServiceResponseType;

int nFrames;
//g2o structures
class SE3Vertex : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SE3Vertex(){}
        void setToOriginImpl() override{
            _estimate = Sophus::SE3d();
        }
        virtual void oplusImpl(const double *_delta){      
            Vector6d delta(_delta);
            _estimate *= Sophus::SE3d::exp(delta); //right multiplication model
        }
        virtual bool read(istream &in){return true;}
        virtual bool write(ostream &out) const {return true;}
};

class SE3CalibrationEdge : public g2o::BaseBinaryEdge<6, Sophus::SE3d, SE3Vertex, SE3Vertex> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Sophus::SE3d Tcb, Twr, Trw;
        SE3CalibrationEdge(Sophus::SE3d _Tcb, Sophus::SE3d _Twr){
            Tcb = _Tcb, Twr = _Twr, Trw = Twr.inverse();
        }
        virtual void computeError(){
            const SE3Vertex *v_cr = static_cast<const SE3Vertex *> (_vertices[0]);
            const SE3Vertex *v_bw = static_cast<const SE3Vertex *> (_vertices[1]);
            const Sophus::SE3d Tcr = v_cr->estimate();
            const Sophus::SE3d Tbw = v_bw->estimate();
            _error = (Tcb*Tbw).log()-(Tcr*Trw).log();
        }
        virtual void linearizeOplus(){
            const SE3Vertex *v_cr = static_cast<const SE3Vertex *> (_vertices[0]);
            const SE3Vertex *v_bw = static_cast<const SE3Vertex *> (_vertices[1]);
            const Sophus::SE3d Tcr = v_cr->estimate();
            const Sophus::SE3d Tbw = v_bw->estimate();
            //calculate jaccobian
            _jacobianOplusXi = -SlamMath::Jr_inv((Tcr*Trw).log())*Twr.Adj();
            _jacobianOplusXj =  SlamMath::Jr_inv((Tcb*Tbw).log());
        }
        virtual bool read(istream &in){return true;}
        virtual bool write(ostream &out) const {return true;}
};

bool ExtrinsicOptimization(ServiceRequestType &req, ServiceResponseType &res);

#endif