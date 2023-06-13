#pragma once

#include <Eigen/Core>
#include <boost/endian/conversion.hpp>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

const std::string slam_config_folder = std::string(getenv("HOME")) + "/slam_configs/";

sensor_msgs::msg::Image matToRosImage(const cv::Mat source, const std_msgs::msg::Header& header,
                                                       const std::string& encoding) {
  sensor_msgs::msg::Image ros_image;
  ros_image.header = header;
  ros_image.height = source.rows;
  ros_image.width = source.cols;
  ros_image.encoding = encoding;
  ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
  ros_image.step = source.cols * source.elemSize();
  size_t size = ros_image.step * source.rows;
  ros_image.data.resize(size);

  if (source.isContinuous()) {
    memcpy((char*)(&ros_image.data[0]), source.data, size);
  } else {
    // Copy by row by row
    uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
    uchar* cv_data_ptr = source.data;
    for (int i = 0; i < source.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += source.step;
    }
  }

  return ros_image;
}

rs2_option getSensorOptions(const rs2::sensor& sensor) {
  std::cout << "Sensor supports the following options:\n" << std::endl;
  for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++) {
    rs2_option option_type = static_cast<rs2_option>(i);
    std::cout << "  " << i << ": " << option_type;
    if (sensor.supports(option_type)) {
      std::cout << std::endl;
      const char* description = sensor.get_option_description(option_type);
      std::cout << "       Description   : " << description << std::endl;
      float current_value = sensor.get_option(option_type);
      std::cout << "       Current Value : " << current_value << std::endl;
    } else {
      std::cout << " is not supported" << std::endl;
    }
  }
  uint32_t selected_sensor_option = 0;
  return static_cast<rs2_option>(selected_sensor_option);
}

std::string getDateString() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  return oss.str();
}

std::string setupOutputFolders(const std::string& date) {
  auto root_path = std::string(getenv("HOME")) + "/ivero_results/";
  auto output_path = root_path + date;

  boost::filesystem::path root_folder = root_path;
  boost::filesystem::path output_folder = output_path;
  boost::filesystem::path depth_folder = output_path + "/depth/";
  boost::filesystem::path rgb_folder = output_path + "/rgb/";

  if (!boost::filesystem::is_directory(root_folder)) { boost::filesystem::create_directory(root_folder); }
  if (!boost::filesystem::is_directory(output_folder)) { boost::filesystem::create_directory(output_folder); }
  if (!boost::filesystem::is_directory(depth_folder)) { boost::filesystem::create_directory(depth_folder); }
  if (!boost::filesystem::is_directory(rgb_folder)) { boost::filesystem::create_directory(rgb_folder); }
  return output_path;
}

std::string getVocabPath() {
  boost::filesystem::path cwd = boost::filesystem::current_path();
  std::string vocab_file = std::string(getenv("HOME")) + "/ORBvoc.txt";
  return vocab_file;
}

std::string getConfigPath(const std::string& serial_number) {
  std::string config_file = slam_config_folder + serial_number + ".yaml";
  return config_file;
}

bool configExists(const std::string& serial_number) {
  std::string config_file = getConfigPath(serial_number);
  return boost::filesystem::exists(config_file);
}

Eigen::Matrix4f getExtrinsics(const rs2::stream_profile& to_stream, const rs2::stream_profile& from_stream) {
  float* Rbc = from_stream.get_extrinsics_to(to_stream).rotation;
  float* tbc = from_stream.get_extrinsics_to(to_stream).translation;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_to_from(Rbc);
  Eigen::Vector3f t_to_from(tbc);
  Eigen::Matrix4f T_to_from = Eigen::Matrix4f::Identity();
  T_to_from.block<3, 3>(0, 0) = R_to_from;
  T_to_from.block<3, 1>(0, 3) = t_to_from;
  return T_to_from;
}

void createConfigFile(const rs2::stream_profile& cam_left, const rs2::stream_profile& cam_right,
                        const std::string& config_file) {
  rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
  rs2_intrinsics intrinsics_right = cam_left.as<rs2::video_stream_profile>().get_intrinsics();

  YAML::Node config;

  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "File.version" << YAML::Value << YAML::DoubleQuoted << "1.0";
  out << YAML::Key << "Camera.type" << YAML::Value << YAML::DoubleQuoted << "Rectified";

  out << YAML::Key << "Camera1.fx" << YAML::Value << intrinsics_left.fx;
  out << YAML::Key << "Camera1.fy" << YAML::Value << intrinsics_left.fy;
  out << YAML::Key << "Camera1.cx" << YAML::Value << intrinsics_left.ppx;
  out << YAML::Key << "Camera1.cy" << YAML::Value << intrinsics_left.ppy;

  out << YAML::Key << "Camera2.fx" << YAML::Value << intrinsics_right.fx;
  out << YAML::Key << "Camera2.fy" << YAML::Value << intrinsics_right.fy;
  out << YAML::Key << "Camera2.cx" << YAML::Value << intrinsics_right.ppx;
  out << YAML::Key << "Camera2.cy" << YAML::Value << intrinsics_right.ppy;

  out << YAML::Key << "Camera.fps" << YAML::Value << 15;
  out << YAML::Key << "Camera.RGB" << YAML::Value << 1;

  out << YAML::Key << "Stereo.b" << YAML::Value << cam_right.get_extrinsics_to(cam_left).translation[0];
  out << YAML::Key << "Stereo.ThDepth" << YAML::Value << std::to_string(40.0);

  out << YAML::Key << "Camera.width" << YAML::Value << intrinsics_left.width;
  out << YAML::Key << "Camera.height" << YAML::Value << intrinsics_left.height;

  out << YAML::Key << "ORBextractor.nFeatures" << YAML::Value << 1250;
  out << YAML::Key << "ORBextractor.scaleFactor" << YAML::Value << std::to_string(1.2);
  out << YAML::Key << "ORBextractor.nLevels" << YAML::Value << 8;
  out << YAML::Key << "ORBextractor.iniThFAST" << YAML::Value << 20;
  out << YAML::Key << "ORBextractor.minThFAST" << YAML::Value << 7;

  out << YAML::Key << "Viewer.KeyFrameSize" << YAML::Value << std::to_string(0.05);
  out << YAML::Key << "Viewer.KeyFrameLineWidth" << YAML::Value << std::to_string(1.0);
  out << YAML::Key << "Viewer.GraphLineWidth" << YAML::Value << std::to_string(0.9);
  out << YAML::Key << "Viewer.PointSize" << YAML::Value << std::to_string(2.0);
  out << YAML::Key << "Viewer.CameraSize" << YAML::Value << std::to_string(0.08);
  out << YAML::Key << "Viewer.CameraLineWidth" << YAML::Value << std::to_string(3.0);
  out << YAML::Key << "Viewer.ViewpointX" << YAML::Value << std::to_string(0.0);
  out << YAML::Key << "Viewer.ViewpointY" << YAML::Value << std::to_string(-0.7);
  out << YAML::Key << "Viewer.ViewpointZ" << YAML::Value << std::to_string(-3.5);
  out << YAML::Key << "Viewer.ViewpointF" << YAML::Value << std::to_string(500.0);
  out << YAML::EndMap;

  if (!boost::filesystem::is_directory(slam_config_folder)) { boost::filesystem::create_directory(slam_config_folder); }

  std::ofstream fout(config_file);
  fout << "%YAML:1.0\n";
  fout << out.c_str();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void setSensorOptions(const std::vector<rs2::sensor>& sensors) {
  int index = 0;
  for (rs2::sensor sensor : sensors) {
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0);
        sensor.set_option(RS2_OPTION_EMITTER_ALWAYS_ON, 0.0);
        sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 1.0);
        sensor.set_option(RS2_OPTION_LASER_POWER, 360);
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
      }
      if (index == 2) { sensor.set_option(RS2_OPTION_EXPOSURE, 100.f); }
    }
  }
}

rclcpp::Time doubleToRostime(const double timestamp) {
  double seconds = std::floor(timestamp);
  double remainder = timestamp - seconds;
  double nanoseconds = std::floor(remainder * 1e9);
  rclcpp::Time time(seconds, nanoseconds);
  return time;
}

sensor_msgs::msg::PointCloud2 pointsToCloud(const std::vector<Eigen::Vector3f>& points, const double timestamp) {
  // figure out number of points
  int numpoints = points.size();
  // declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = doubleToRostime(timestamp);
  cloud_msg.header.frame_id = "slam_frame";
  cloud_msg.width = numpoints;
  cloud_msg.height = 1;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;

  // set field
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(numpoints);

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

  for (const auto& p : points) {
    *out_x = p.x();
    *out_y = p.y();
    *out_z = p.z();
    ++out_x;
    ++out_y;
    ++out_z;
  }
  return cloud_msg;
}