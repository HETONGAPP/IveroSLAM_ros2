cmake_minimum_required(VERSION 3.5)
project(iveroslam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iveroslam_interfaces REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZIPPER REQUIRED zipper)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(realsense2)
find_package(yaml-cpp)
find_package(OpenCV 4.2)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams)

 # this should point to where IveroSLAM is on the system
set(ORB_SLAM3_DIR $ENV{HOME}/projects/IveroSLAM)

include_directories(
  include
  ${PROJECT_NAME}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

set(LIBS
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${realsense2_LIBRARY}
  ${ZIPPER_LIBRARIES}
  -lboost_filesystem
  -lboost_iostreams
)

set(ROS_LIBS
  rclcpp 
  std_srvs 
  std_msgs 
  nav_msgs
  sensor_msgs
  geometry_msgs
  iveroslam_interfaces
)

add_executable(ivero_slam src/ivero_slam.cc)
ament_target_dependencies(ivero_slam ${ROS_LIBS})
target_link_libraries(ivero_slam ${LIBS})
set_target_properties(ivero_slam PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS ivero_slam 
        DESTINATION lib/${PROJECT_NAME})

ament_package()
