#include <iveroslam_ros2/CallbackHarness.h>
#include <iveroslam_ros2/utils.h>

std::string CallbackHarness::output_path;
std::mutex CallbackHarness::callback_mutex;
std::condition_variable CallbackHarness::cond_image_rec;
cv::Mat CallbackHarness::imCV, CallbackHarness::imRightCV;
double CallbackHarness::timestamp_image;
bool CallbackHarness::image_ready;
bool CallbackHarness::output_rgbd;
int CallbackHarness::count_im_buffer;
double CallbackHarness::prev_depthkf_timestamp;

std::optional<RGBDImage> CallbackHarness::getRGBD(const rs2::frameset& frameset) {
  rs2::align align(RS2_STREAM_COLOR);
  // align frameset to color stream
  auto aligned_fs = align.process(frameset);
  auto timestamp = frameset.get_timestamp() * 1e-3;

  rs2::video_frame rgb_frame = frameset.get_color_frame();
  rs2::depth_frame depth_frame = aligned_fs.get_depth_frame();

  if (rgb_frame.get_data_size() <= 0 || depth_frame.get_data_size() <= 0) {
    std::cout << std::fixed << "No data in rgbd stream: " << timestamp << std::endl;
    return {};
  }

  rs2::spatial_filter spat;
  spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4.0);
  rs2::threshold_filter thr(0.15f, 3.0f);
  rs2::temporal_filter temp;
  temp.set_option(RS2_OPTION_HOLES_FILL, 0.0);
  rs2::disparity_transform depth2disparity;
  rs2::disparity_transform disparity2depth(false);

  depth_frame = depth2disparity.process(depth_frame);
  depth_frame = spat.process(depth_frame);
  depth_frame = temp.process(depth_frame);
  depth_frame = disparity2depth.process(depth_frame);
  depth_frame = thr.process(depth_frame);

  RGBDImage rgbd_image;
  rgbd_image.timestamp = timestamp;

  cv::Mat depth_raw =
      cv::Mat(depth_frame.get_height(), depth_frame.get_width(), CV_16SC1, const_cast<void*>(depth_frame.get_data()));
  cv::Mat depth_metric_mm;
  depth_raw.convertTo(depth_metric_mm, CV_32FC1);
  depth_metric_mm *= depth_frame.get_units() * 1000.0;
  cv::Mat depth_out;
  depth_metric_mm.convertTo(depth_out, CV_16UC1);
  rgbd_image.depth = depth_out;

  cv::Mat rgb = cv::Mat(cv::Size(rgb_frame.get_width(), rgb_frame.get_height()), CV_8UC3, (void*)(rgb_frame.get_data()),
                        cv::Mat::AUTO_STEP);
  rgbd_image.rgb = rgb;

  return rgbd_image;
}

void CallbackHarness::callback(const rs2::frame& frame) {
  std::unique_lock<std::mutex> lock(callback_mutex);

  if (rs2::frameset fs = frame.as<rs2::frameset>()) {
    assert(fs.supports_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_EMITTER_MODE) &&
           "Metadata not supported, cannot perform laser emitter switching! Build librealsense with RSUSB backend.");

    // metadata is delayed for this? i.e. if it says its off its bc the previous frame was off?
    auto emitter_on = fs.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
    if (emitter_on == 0 && CallbackHarness::output_rgbd) {
      auto cur_timestamp = fs.get_timestamp() / 1000.0;
      if (cur_timestamp - prev_depthkf_timestamp < 0.25) { return; }
      prev_depthkf_timestamp = cur_timestamp;

      auto rgbd_image = getRGBD(fs);

      if (!rgbd_image.has_value()) { return; }

      const auto filename = std::to_string(rgbd_image.value().timestamp) + ".png";
      if (output_path.empty()) { return; }

      cv::imwrite(output_path + "/depth/" + filename, rgbd_image.value().depth);
      cv::imwrite(output_path + "/rgb/" + filename, rgbd_image.value().rgb);
      return;
    }

    count_im_buffer++;

    double new_timestamp_image = fs.get_timestamp() * 1e-3;
    if (abs(timestamp_image - new_timestamp_image) < 0.001) {
      count_im_buffer--;
      return;
    }

    rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
    rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

    imCV = cv::Mat(cv::Size(ir_frameL.get_width(), ir_frameL.get_height()), CV_8U, (void*)(ir_frameL.get_data()),
                   cv::Mat::AUTO_STEP);
    imRightCV = cv::Mat(cv::Size(ir_frameR.get_width(), ir_frameR.get_height()), CV_8U, (void*)(ir_frameR.get_data()),
                        cv::Mat::AUTO_STEP);

    timestamp_image = fs.get_timestamp() * 1e-3;
    image_ready = true;

    lock.unlock();
    cond_image_rec.notify_all();
  }
}