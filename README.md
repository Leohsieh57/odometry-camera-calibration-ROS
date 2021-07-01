# Odometry Camera Calibration ROS
This package allows you to simultaneously estimate  

  - **camera intrinsic parameters**  
  - **robot-camera coordinate transformation**


# 1. Dependencies & Environment
This package runs on ROS-noetic with the following dependencies:  

  - numpy  
  - scipy(1.17.4)  
  - Eigen3  
  - Sophus  
  - g2o(embedded in this repo)  

For g2o dependency, go to `/thirdparty/g2o` and type

    mkdir build  
    cd build  
    cmake ..  
    make -j8  
    roscd  
    cd ..  
    catkin_make
   
> ⚠️ ***For old-version ROS users with python2.7***  
>You have to set python path back to 3.5 since scipy 1.17.4 doesn't support python2.7

# 2. Brief Guide-through
Type the following command to check out package features first

    roslaunch odometry_camera_calibration visualize_calibration_results.launch
Outcome will be my calibration result, based on experimental data under `/calibration_data` 

# 3. Parameters
  - **odometry_camera_calibration/checkerBoardParam/nRows**  
  Row number of your checkerboard 
  - **odometry_camera_calibration/checkerBoardParam/nCols**  
  Column number of your checkerboard
  - **odometry_camera_calibration/checkerBoardParam/nFrames**  
  Number of images you want to use for checkerboard calibration

	> ⚠️ ***Important***  
	> A large number may result in a really long calculation time
  - **odometry_camera_calibration/checkerBoardParam/cellSize**  
  Size of your checkerboard grids
  
# 4. Tutorials
## 4-1. collect calibration datas
Prepare a checkerboard and set up all parameters mentioned above  
  
    roslaunch odometry_camera_calibration start_calibration.launch
    
Now guide your robot around the fixed checkerboard to collect data  
The terminal will contantly show `currently x frames`  
After collected sufficient frames, the node will call `CalibrationServer` for estimation

## 4-2. calibration results
The result will be saved as `calibration_results.yaml` under `/params`

## 4-3. visualize calibration results
Check if you can find `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`  
If four of them are successfully generated, execute:
  
    roslaunch odometry_camera_calibration visualize_calibration_results.launch
    
This gives you a concrete image on how good your calibration went
# 5. Launch File Usage
## 5-1. start_calibration.launch
  - Subscribes to **/robot_pose** for gmapping robot pose with `nav_msgs/Odometry`
  - Subscribes to **/camera_image** for camera image with `sensor_msgs/Image`
  - Generates `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`
  - Generates `calibration_results.yaml` under `/params`
  
## 5-2. start_calibration_offline.launch
**This launchfile is mostly for development debugging, you probably won't need it**
  - Requires `camera_odom.npy laser_odom.npy` under `/calibration_data`
  - Generates `Tcr.npy Tbw.npy` under `/calibration_data`

## 5-3. visualize_calibration_results.launch
  - Requires `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`
  - Publishes **/calibrated_camera_pose** for rviz visualization
  - Publishes **/robot_pose** for rviz visualization
  
