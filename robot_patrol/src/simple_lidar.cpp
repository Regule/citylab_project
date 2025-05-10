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

void ScanVector::update(const std::vector<float> updated_scan) {
  scan_ = updated_scan;
}

void ScanVector::clear() { scan_.clear(); }

float ScanVector::operator[](int idx) const {
  int size = static_cast<int>(scan_.size());
  if (size == 0) {
    throw std::out_of_range("Attempting to access empty scan vector.");
  }
  while (idx < 0) {
    idx += size;
  }
  while (idx >= size) {
    idx -= size;
  }
  return scan_[idx];
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
  scan_.update(msg->ranges);
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
  std::pair<int, int> range = get_index_range_for_cone_(angle, cone_size);
  if (range.first > range.second) {
    measurement.state = LidarMeasurement::OUT_OF_RANGE;
    return measurement;
  }

  int min_idx = 0;
  float min_val = std::numeric_limits<float>::infinity();
  bool readout_valid = false;
  for (int idx = range.first; idx < range.second; ++idx) {
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
  std::pair<int, int> range = get_index_range_for_cone_(angle, cone_size);
  if (range.first > range.second) {
    measurement.state = LidarMeasurement::OUT_OF_RANGE;
    return measurement;
  }

  int min_idx = 0;
  float min_val = 0.0f;
  bool readout_valid = false;
  for (int idx = range.first; idx < range.second; ++idx) {
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
LidarMeasurement SimpleLidar::get_sum(float angle, float cone_size) const {
  LidarMeasurement measurement;
  std::pair<int, int> range = get_index_range_for_cone_(angle, cone_size);
  if (range.first > range.second) {
    measurement.state = LidarMeasurement::OUT_OF_RANGE;
    return measurement;
  }

  float sum = 0.0f;
  bool readout_valid = false;
  for (int idx = range.first; idx < range.second; ++idx) {
    if (std::isnan(scan_[idx]) || std::isinf(scan_[idx]))
      continue;
    readout_valid = true;
    sum += scan_[idx];
  }
  if (readout_valid) {
    measurement.state = LidarMeasurement::OK;
    measurement.distance = sum;
    measurement.angle = angle;
  } else {
    measurement.state = LidarMeasurement::ERROR;
  }
  return measurement;
}

float SimpleLidar::degree_to_radian(float degree) {
  return degree * 0.0174532925f;
}

std::pair<int, int>
SimpleLidar::get_index_range_for_cone_(float angle, float cone_size) const {
  std::pair<int, int> range;
  range.first =
      static_cast<int>((angle - cone_size / 2 - cfg_->angle_min) / cfg_->step);
  range.second =
      static_cast<int>((angle + cone_size / 2 - cfg_->angle_min) / cfg_->step) +
      1;
  if (angle - cone_size / 2 < cfg_->angle_min ||
      angle + cone_size / 2 >=
          cfg_->angle_min + cfg_->step * cfg_->sample_count) {
    range.first = 0;
    range.second = -1;
  }

  return range;
}

} // namespace citylab