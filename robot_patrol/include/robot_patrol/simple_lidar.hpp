#ifndef CITYLAB_SIMPLE_LIDAR_HPP
#define CITYLAB_SIMPLE_LIDAR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace citylab {
struct LidarMeasurement {
  enum State { OK, TIMEOUT, OUT_OF_RANGE, ERROR };
  State state = ERROR;
  bool condition = false;
  float angle = 0.0;
  float distance = 0.0;

  std::string str() const;
  static std::string state_to_str(State state);
};

struct LidarConfig {
  float angle_min;
  float step;
  int sample_count;
  std::pair<float, float> range;
};

class ScanVector {
public:
  ScanVector() = default;
  ~ScanVector() = default;
  void update(const std::vector<float> updated_scan);
  void clear();
  float operator[](int idx) const;

private:
  std::vector<float> scan_;
};

class SimpleLidar {
public:
  SimpleLidar() = default;
  explicit SimpleLidar(const LidarConfig &cfg);
  ~SimpleLidar() = default;

  void update(sensor_msgs::msg::LaserScan::SharedPtr msg);

  LidarMeasurement get_closest_range(float angle, float cone_size) const;
  LidarMeasurement get_farthest_range(float angle, float cone_size) const;
  LidarMeasurement get_sum(float angle, float cone_size) const;

  static float degree_to_radian(float degree);

private:
  std::unique_ptr<LidarConfig> cfg_;
  ScanVector scan_;

  void build_config_(sensor_msgs::msg::LaserScan::SharedPtr msg);
  std::pair<int, int> get_index_range_for_cone_(float angle,
                                                float cone_size) const;
};

} // namespace citylab

#endif // CITYLAB_SIMPLE_LIDAR_HPP