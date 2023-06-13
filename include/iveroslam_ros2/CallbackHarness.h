#pragma once

#include <optional>
#include <thread>
#include <condition_variable>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>

/// @brief Struct representing a single RGBD image
struct RGBDImage {
  cv::Mat rgb;
  cv::Mat depth;
  double timestamp;
};

/// @brief This is a fully static class to encapsulate the callback for the realsense device and how it interacts with
/// orb slam
class CallbackHarness {
public:
  /// @brief Gets the RGB and aligned Depth image from a frameset
  /// @param frameset to retrieve RGBD from
  /// @return <RGB, Depth>
  static std::optional<RGBDImage> getRGBD(const rs2::frameset& frameset);

  /// @brief Main callback for realsense device
  /// @param frame to process
  static void callback(const rs2::frame& frame);

  static std::string output_path;
  /// @brief
  static std::mutex callback_mutex;
  /// @brief
  static std::condition_variable cond_image_rec;
  /// @brief
  static cv::Mat imCV, imRightCV;
  /// @brief
  static double timestamp_image;
  /// @brief
  static bool image_ready;
  /// @brief
  static int count_im_buffer; // count dropped frames
  /// @brief
  static double prev_depthkf_timestamp;
  /// @brief 
  static bool output_rgbd;

private:
  // Disallow creating an instance of this object
  CallbackHarness() {}
};

#include "CallbackHarnessImp.hpp"