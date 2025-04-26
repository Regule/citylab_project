#ifndef SIMPLE_LIDAR_HPP
#define SIMPLE_LIDAR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>

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
  std::vector<float> scan_;

  void build_config_(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

} // namespace citylab

#endif // SIMPLE_LIDAR_HPP