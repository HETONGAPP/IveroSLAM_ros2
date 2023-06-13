
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <zipper/zipper.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <System.h>

#include <iveroslam_ros2/CallbackHarness.h>
#include <iveroslam_ros2/utils.h>

#include <rclcpp/rclcpp.hpp>

#include <iveroslam_interfaces/srv/system_status.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iveroslam_interfaces/msg/system_status.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

enum class State { Started, Stopped, Paused };

State current_state;
bool output_results;
bool continue_session;
double session_start_time = -1.0;
iveroslam_interfaces::msg::SystemStatus current_status;
std::mutex mtx;

void exit_loop_handler(int s) {
  std::cout << "\nExiting IveroSLAM, wrapping up all processes." << std::endl;
  continue_session = false;
}

void pauseScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Paused;
  response_msg = "Paused";
  response->success = true;
  response->message = response_msg;
  lk.unlock();
}

void startScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Started;
  output_results = true;
  response_msg = "Started";
  response->success = true;
  response->message = response_msg;
  lk.unlock();
}

void stopScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Stopped;
  output_results = true;
  response_msg = "Stopped";
  response->success = true;
  response->message = response_msg;
  lk.unlock();
}

void cancelScan(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  std::string response_msg;
  current_state = State::Stopped;
  output_results = false;
  response_msg = "Canceled";
  response->success = true;
  response->message = response_msg;
  lk.unlock();
}

void getStatus(const std::shared_ptr<iveroslam_interfaces::srv::SystemStatus::Request> request,
               std::shared_ptr<iveroslam_interfaces::srv::SystemStatus::Response> response) {
  std::unique_lock<std::mutex> lk(mtx);
  rclcpp::Time timestamp = current_status.header.stamp;
  response->timestamp = timestamp.seconds();
  response->session_start_time = session_start_time;
  response->tracking_status = current_status.tracking_status;
  response->is_lost = current_status.is_lost;
  response->map_changed = current_status.map_changed;
  if (current_state == State::Started) {
    response->started = true;
  } else {
    response->started = false;
  }
  response->success = true;
  lk.unlock();
}

std::string getStateString() {
  if (current_state == State::Stopped) {
    return "\033[1;31mStopped\033[0m";
  } else if (current_state == State::Started) {
    return "\033[1;32mStarted\033[0m";
  } else if (current_state == State::Paused) {
    return "\033[1;33mPaused\033[0m";
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ivero_slam_node");

  // services for controlling the state
  auto start_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/start", &startScan);
  auto stop_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/stop", &stopScan);
  auto cancel_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/cancel", &cancelScan);
  auto pause_service = node->create_service<std_srvs::srv::Trigger>("/ivero_slam/pause", &pauseScan);
  auto status_service =
      node->create_service<iveroslam_interfaces::srv::SystemStatus>("/ivero_slam/get_status", &getStatus);

  // publishers
  auto image_pub = node->create_publisher<sensor_msgs::msg::Image>("/ivero_slam/tracked_image", 10);
  auto output_path_pub = node->create_publisher<std_msgs::msg::String>("/ivero_slam/output_path", 10);
  auto status_pub = node->create_publisher<iveroslam_interfaces::msg::SystemStatus>("/ivero_slam/system_status", 10);
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ivero_slam/map_points", 10);
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/ivero_slam/pose", 10);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/ivero_slam/trajectory", 10);

  // sigint handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_loop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // initial states
  current_state = State::Started;
  output_results = true;
  continue_session = true;

  // look for realsense devices
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  assert(devices.size() > 0 && "No device connected, please connect a RealSense device.");
  rs2::device selected_device = devices[0];

  // assert camera type
  auto camera_name = std::string(selected_device.get_info(RS2_CAMERA_INFO_NAME));
  assert(camera_name == "Intel RealSense D455" && "Device must be an Intel RealSense D455.");

  // set sensor options
  setSensorOptions(selected_device.query_sensors());

  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 30);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  rs2::pipeline_profile pipe_profile = pipe.start(cfg, CallbackHarness::callback);

  // Get respective streams for automatic config file creation
  rs2::stream_profile cam_left = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
  rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);

  // get extrinsic from left camera to color camera
  Eigen::Matrix4f T_rgb_camleft = getExtrinsics(pipe_profile.get_stream(RS2_STREAM_COLOR), cam_left);

  // get config file, create if it doesnt exist
  auto serial_number = std::string(selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  std::string config_file = getConfigPath(serial_number);
  if (!boost::filesystem::exists(config_file)) { createConfigFile(cam_left, cam_right, config_file); }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(getVocabPath(), config_file, ORB_SLAM3::System::STEREO, false, 0, getDateString());
  float imageScale = SLAM.GetImageScale();

  // variables for orb slam
  double timestamp;
  cv::Mat im, imRight;

  // setup callback variables
  CallbackHarness::timestamp_image = -1.0;
  CallbackHarness::image_ready = false;
  CallbackHarness::count_im_buffer = 0;
  CallbackHarness::prev_depthkf_timestamp = 0;
  rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
  CallbackHarness::output_rgbd = false;

  bool slam_reset = true;
  bool localization_only = false;

  std::set<double> lost_timestamps;

  std::string output_path;
  std::string traj_file_name;

  double previous_state_update = 0.0;
  double state_update_period = 3.0;

  while (continue_session) {
    rclcpp::spin_some(node);

    // access current system state safely
    std::unique_lock<std::mutex> lk(CallbackHarness::callback_mutex);
    auto state = current_state;
    lk.unlock();

    auto now = node->now().seconds();
    if (now - previous_state_update > state_update_period) {
      RCLCPP_INFO(node->get_logger(), "Current system state: %s", getStateString().c_str());
      previous_state_update = now;
    }

    if (state == State::Stopped) {
      if (!slam_reset) {
        CallbackHarness::output_rgbd = false; // stop outputting rgbd images
        // publish output folder path
        if (output_results) {
          SLAM.SaveTrajectoryTUMTransformed(traj_file_name, T_rgb_camleft, session_start_time, lost_timestamps);
          std::string output_folder_zipped = CallbackHarness::output_path + ".zip";
          zipper::Zipper zipper(output_folder_zipped);
          zipper.add(CallbackHarness::output_path);
          zipper.close();
          // delete folder after compressing
          boost::system::error_code ec;
          boost::filesystem::remove_all(CallbackHarness::output_path, ec);
          if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }
          // publish absolute path to compressed folder
          std_msgs::msg::String output_path_message;
          output_path_message.data = output_folder_zipped;
          output_path_pub->publish(output_path_message);
        } else {
          // delete folder
          boost::system::error_code ec;
          boost::filesystem::remove_all(CallbackHarness::output_path, ec);
          if (ec) { std::cerr << "Error deleting directory: " << ec.message() << std::endl; }
        }
        slam_reset = true;
      }

      // continually reset data
      CallbackHarness::count_im_buffer = 0;
      CallbackHarness::image_ready = false;
      session_start_time = -1.0;
      // sleep for a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

    } else {
      // if paused, we only want to localize
      if (current_state == State::Paused && !localization_only) {
        SLAM.ActivateLocalizationMode();
        localization_only = true;
      } else if (current_state == State::Started && localization_only) {
        SLAM.DeactivateLocalizationMode();
        localization_only = false;
      }

      if (slam_reset) {
        CallbackHarness::output_rgbd = true; // start outputting rgbd images
        session_start_time = now;
        slam_reset = false;
        output_path = setupOutputFolders(getDateString());
        traj_file_name = output_path + "/trajectory.txt";
        CallbackHarness::output_path = output_path;
        lost_timestamps.clear();
      }

      std::unique_lock<std::mutex> lk(CallbackHarness::callback_mutex);
      if (!CallbackHarness::image_ready) CallbackHarness::cond_image_rec.wait(lk);

      if (CallbackHarness::count_im_buffer > 1) {
        std::cout << "\033[1;31m" << CallbackHarness::count_im_buffer - 1 << " dropped frames.\033[0m" << std::endl;
      }
      CallbackHarness::count_im_buffer = 0;

      // Copy the data
      timestamp = CallbackHarness::timestamp_image;
      im = CallbackHarness::imCV.clone();
      imRight = CallbackHarness::imRightCV.clone();
      lk.unlock();

      // Image no longer ready for next iteration
      CallbackHarness::image_ready = false;

      if (imageScale != 1.f) {
        int width = im.cols * imageScale;
        int height = im.rows * imageScale;
        cv::resize(im, im, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));
      }

      // create message header
      std_msgs::msg::Header header;
      header.stamp = doubleToRostime(timestamp);
      header.frame_id = "left_ir";

      // Stereo images are already rectified.
      const auto Tcw = SLAM.TrackStereo(im, imRight, timestamp);

      // publish system status
      current_status.header = header;
      current_status.tracking_status = SLAM.GetTrackingState();
      current_status.is_lost = SLAM.isLost();
      current_status.map_changed = SLAM.MapChanged();
      status_pub->publish(current_status);
      if (current_status.map_changed) {
        // todo: publish trajectory if map changed (make function in system to get full trajectory)
      }
      if(current_status.tracking_status != 2){
        lost_timestamps.insert(timestamp);
      }

      // crop tracked image to be same size as input
      cv::Mat tracked_frame = SLAM.GetDrawnFrame();
      cv::Rect myROI(0, 0, 640, 480);
      cv::Mat croppedImage = tracked_frame(myROI);
      auto image_msg = matToRosImage(tracked_frame(myROI), header, "bgr8");
      image_pub->publish(image_msg);

      auto points = SLAM.GetSparsePointCloud();
      auto pointcloud_msg = pointsToCloud(points, timestamp);
      cloud_pub->publish(pointcloud_msg);

      Eigen::Vector3f t = Tcw.translation();
      Eigen::Quaternionf q = Tcw.unit_quaternion();
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = header;
      pose_msg.pose.position.x = t.x();
      pose_msg.pose.position.y = t.y();
      pose_msg.pose.position.z = t.z();
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.z = q.z();
      pose_pub->publish(pose_msg);
    }
  }
}