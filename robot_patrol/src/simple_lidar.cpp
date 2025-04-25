#include "robot_patrol/simple_lidar.hpp"

namespace citylab {
std::string LidarMeasurement::str() const {
  std::stringstream text;
  text << "State:" << state_to_str(state) << " ";
  text << "Condition:" << (condition ? "TRUE" : "FALSE") << " ";
  text << "Angle:" << std::setprecision(3) << angle << " ";
  text << "Distance:" << std::setprecision(3) << distance << " ";
  std::string str = text.str();
  return str;
}

std::string LidarMeasurement::state_to_str(State state) {
  switch (state) {
  case OK:
    return std::string("OK");
  case TIMEOUT:
    return std::string("TIMEOUT");
  case OUT_OF_RANGE:
    return std::string("OUT_OF_RANGE");
  case ERROR:
    return std::string("ERROR");
  default:
    return std::string("WTF");
  }
}

SimpleLidar::SimpleLidar(const LidarConfig &cfg) {
  cfg_ = std::make_unique<LidarConfig>(cfg);
}

void SimpleLidar::update(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!cfg_) {
    build_config_(msg);
  }
  int sample_count = msg->ranges.size();
  if (sample_count != cfg_->sample_count) {
    RCLCPP_WARN(rclcpp::get_logger("lidar_awareness"),
                "Recieved %d samples while expected %d.", sample_count,
                cfg_->sample_count);
    scan_.clear();
    return;
  }
  scan_ = msg->ranges;
}

void SimpleLidar::build_config_(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  cfg_ = std::make_unique<LidarConfig>();
  cfg_->angle_min = msg->angle_min;
  cfg_->step = msg->angle_increment;
  cfg_->sample_count = msg->ranges.size();
  cfg_->range.first = msg->range_min;
  cfg_->range.second = msg->range_max;
}

LidarMeasurement SimpleLidar::get_closest_range(float angle,
                                                float cone_size) const {
  LidarMeasurement measurement;
  int initial_sample =
      static_cast<int>((angle - cone_size / 2 - cfg_->angle_min) / cfg_->step);
  int end_sample =
      static_cast<int>((angle + cone_size / 2 - cfg_->angle_min) / cfg_->step) +
      1;
  RCLCPP_INFO(rclcpp::get_logger("lidar_awareness"), "Range %d - %d",
              initial_sample, end_sample);
  if (initial_sample < 0 || initial_sample + end_sample > cfg_->sample_count) {
    measurement.state = LidarMeasurement::OUT_OF_RANGE;
    return measurement;
  }

  int min_idx = 0;
  float min_val = std::numeric_limits<float>::infinity();
  bool readout_valid = false;
  for (int idx = initial_sample; idx < end_sample; ++idx) {
    if (std::isnan(scan_[idx]))
      continue;
    readout_valid = true;
    if (scan_[idx] < min_val) {
      min_val = scan_[idx];
      min_idx = idx;
    }
  }
  if (readout_valid) {
    measurement.state = LidarMeasurement::OK;
    measurement.distance = min_val;
    measurement.angle = cfg_->angle_min + cfg_->step * min_idx;
  } else {
    measurement.state = LidarMeasurement::ERROR;
  }
  return measurement;
}

LidarMeasurement SimpleLidar::get_farthest_range(float angle,
                                                 float cone_size) const {
  LidarMeasurement measurement;
  int initial_sample =
      static_cast<int>((angle - cone_size / 2 - cfg_->angle_min) / cfg_->step);
  int end_sample =
      static_cast<int>((angle + cone_size / 2 - cfg_->angle_min) / cfg_->step) +
      1;
  if (initial_sample < 0 || initial_sample + end_sample > cfg_->sample_count) {
    measurement.state = LidarMeasurement::OUT_OF_RANGE;
    return measurement;
  }

  int min_idx = 0;
  float min_val = 0.0f;
  bool readout_valid = false;
  for (int idx = initial_sample; idx < end_sample; ++idx) {
    if (std::isnan(scan_[idx]) || std::isinf(scan_[idx]))
      continue;
    readout_valid = true;
    if (scan_[idx] > min_val) {
      min_val = scan_[idx];
      min_idx = idx;
    }
  }
  if (readout_valid) {
    measurement.state = LidarMeasurement::OK;
    measurement.distance = min_val;
    measurement.angle = cfg_->angle_min + cfg_->step * min_idx;
  } else {
    measurement.state = LidarMeasurement::ERROR;
  }
  return measurement;
}
} // namespace citylab