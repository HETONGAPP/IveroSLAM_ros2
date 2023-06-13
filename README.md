# IveroSLAM_ros2

This repository wraps IveroSLAM in a ros package so we can publish results over ros. This runs exactly the same as the original IveroSLAM, except it will publish tracked images and point clouds to ros.

# Building and Running IveroSLAM_ros2

1. Follow install instructions in [IveroSLAM](https://github.com/Luxolis/IveroSLAM)
2. `git clone https://github.com/Luxolis/IveroSLAM_ros2.git ~/ros2_ws/src`
3. `git clone https://github.com/Luxolis/IveroSLAM_interfaces.git ~/ros2_ws/src`
4. Replace line 22 in CMakeLists.txt: `set(ORB_SLAM3_DIR $ENV{HOME}/projects/IveroSLAM)` with the path to IveroSLAM if its not already correct
5. `colcon build` in the ros2_ws directory
6. Make sure a realsense d455 is plugged in and the repository has been built then run: `ros2 run iveroslam_ros2 ivero_slam`