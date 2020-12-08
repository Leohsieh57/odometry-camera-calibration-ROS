#include <CalibrationServer.h>

bool ExtrinsicOptimization(ServiceRequestType &req,  ServiceResponseType &res){
    vector<Sophus::SE3d> TcbVec = SlamMath::toSE3d(req.cameraOdom);
    vector<Sophus::SE3d> TwrVec = SlamMath::toSE3d(req.laserOdom);

    g2o::SparseOptimizer optimizer;
    LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();
    BlockSolverType* solver_ptr = new BlockSolverType(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);
    SE3Vertex *v_cr = new SE3Vertex();
    SE3Vertex *v_bw = new SE3Vertex();
    v_cr->setId(0);
    v_bw->setId(1);
    optimizer.addVertex(v_cr);
    optimizer.addVertex(v_bw);
    for (int i = 0; i < TcbVec.size(); i++){
        Sophus::SE3d Tcb = TcbVec[i];
        Sophus::SE3d Twr = TwrVec[i];
        SE3CalibrationEdge *e = new SE3CalibrationEdge(Tcb, Twr);
        e->setId(i);
        e->setVertex(0,v_cr);
        e->setVertex(1,v_bw);
        e->setInformation(Matrix6d::Identity());
        e->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(e);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(400);
    Sophus::SE3d Tcr = v_cr->estimate();
    Sophus::SE3d Tbw = v_bw->estimate();
    cout << "estimated Tcr: "  << endl << Tcr.matrix() << endl;
    res.Tcr = SlamMath::toOdometry(Tcr);
    res.Tbw = SlamMath::toOdometry(Tbw);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "extrisic_optimization_service");
    ros::NodeHandle n;
    n.getParam("odometry_camera_calibration/checkerBoardParam/nFrames", nFrames);
    ros::ServiceServer service = n.advertiseService("calibration_server", ExtrinsicOptimization);
    ros::spin();

    return 0;
}
