# ARTSLAM
***artslam_laser_3d*** is an open source c++ package for accurate real-time 6DoF SLAM, using a 3D LIDAR. It works on full point clouds and it is a graph-based system. Tracking is achieved using scan-to-keyframe matching, where keyframes correspond to point clouds taken after a certain distance is traveled. Loop detection involves three steps: range based filtering, Scan Context and keyframe-to-keyframe alignment. ART-SLAM also supports several graph constraints, such as floor/ground coefficients, IMU orientation and GNSS data.

A basic ROS wrapper is provided at [[link]](https://github.com/MatteoF94/ARTSLAM_WRAPPER).

## Requirements
***artslam_laser_3d*** requires the following libraries:

- Eigen3
- Boost > 1.65.1
- PCL > 1.10
- OpenCV > 4.0
- g2o
- suitesparse

Moreover, it requires the following packages:

- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
- [artslam_core](https://github.com/MatteoF94/ARTSLAM_CORE)

## Build
***artslam_laser_3d*** is built using catkin, although ROS is not mandatory:
```bash
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/MatteoF94/ARTSLAM_CORE.git
git clone https://github.com/MatteoF94/ARTSLAM.git
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
```
Alternatively to catkin_make, you can use the "catkin build artslam_laser_3d" approach, which will automatically build all packages. 

## Instructions
The file applications/artslam_laser3d_offline.cpp describes in detail how to build your own SLAM system, step by step. Two configuration files are also included to work on the KITTI dataset. There are plenty of parameters, but the majority does not need to be tuned. If you are interested in the use of each parameter, you can check the corresponding header (e.g., tracker --> tracker.h), where they are briefly explained. 

## Papers
Frosi Matteo, and Matteo Matteucci. "ART-SLAM: Accurate Real-Time 6DoF LiDAR SLAM." IEEE Robotics and Automation Letters (2022). [[link]](https://ieeexplore.ieee.org/abstract/document/9691876).
