# odometry camera calibration ROS
This package allows you to simultaneously estimate  
  - **camera intrinsic parameters**
  - **robot-camera coordinate transformation**
[![Alt text](https://github.com/Leohsieh57/odometry-camera-calibration-ROS/blob/main/readme_pictures/demo.png)](https://youtu.be/0YEWkK3-Adk)  
If you find this package useful, please subsrcibe to Haachama youtube channel  
https://www.youtube.com/channel/UC1CfXB_kRs3C-zaeTG3oGyg

# 1. dependencies & environment
This package runs on ROS-noetic with the following dependencies:
  - numpy
  - scipy(1.17.4)
  - Eigen3
  - Sophus
  - g2o(embedded in this repo)  

for g2o dependency, go to `/thirdparty/g2o` and type

    mkdir build  
    cd build  
    cmake ..  
    make -j8  
    roscd  
    cd ..  
    catkin_make
   
> ⚠️ ***for old-version ROS users with python2.7***  
>You have to set python path back to 3.5 since scipy1.17.4 doesn't support python2.7

# 2. brief guide-through
Type the following command to check out package features first

    roslaunch odometry_camera_calibration visualize_calibration_results.launch
outcome will be my calibration result, based on experimental data under `/calibration_data` 

# 3. parameters
  - **odometry_camera_calibration/checkerBoardParam/nRows**  
  row number of your checkerboard 
  - **odometry_camera_calibration/checkerBoardParam/nCols**  
  column number of your checkerboard
  - **odometry_camera_calibration/checkerBoardParam/nFrames**  
  number of images you want to use for checkerboard calibration    
    > ⚠️ ***a large number may result in a really long calculation time***  
  - **odometry_camera_calibration/checkerBoardParam/cellSize**  
  size of your checkerboard grids
  
# 4. tutorials
## 1. collect calibration datas
prepare a checkerboard and set up all parameters mentioned above  
  
    roslaunch odometry_camera_calibration start_calibration.launch
    
now guide your robot around the fixed checkerboard to collect data  
the terminal will contantly show `currently x frames`  
after collected sufficient frames, the node will call `CalibrationServer` for estimation

## 2. calibration results
the result will be saved as `calibration_results.yml` under `/params`

## 3. visualize calibration results
check if you can find `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`  
if four of them are successfully generated, execute:
  
    roslaunch odometry_camera_calibration visualize_calibration_results.launch
    
this gives you a concrete image on how good your calibration went
# 5. launchfile usage
## 1. start_calibration.launch
  - subscribes to **/robot_pose** for gmapping robot pose with `nav_msgs/Odometry`
  - subscribes to **/camera_image** for camera image with `sensor_msgs.msg/Image`
  - generates `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`
  - generates `calibration_results.yml` under `/params`
  
## 2. start_calibration_offline.launch
**this launchfile is mostly for development debugging, you probably won't need it**
  - requires `camera_odom.npy laser_odom.npy` under `/calibration_data`
  - generates `Tcr.npy Tbw.npy` under `/calibration_data`

## 3. visualize_calibration_results.launch
  - requires `camera_odom.npy laser_odom.npy Tcr.npy Tbw.npy` under `/calibration_data`
  - publishes **/calibrated_camera_pose** for rviz visualization
  - publishes **/robot_pose** for rviz visualization
  
