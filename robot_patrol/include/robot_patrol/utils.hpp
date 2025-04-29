#ifndef CITYLAB_UTILS_H
#define CITYLAB_UTILS_H

#include <cmath>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace citylab {
struct Position2D {
  double x;
  double y;
  double theta;

  static Position2D from_odometry(const nav_msgs::msg::Odometry &msg);
  static Position2D from_pose2D(const geometry_msgs::msg::Pose2D &msg);

  geometry_msgs::msg::Pose2D to_Pose2D() const;
  geometry_msgs::msg::Twist to_Twist() const;
  std::string to_str() const;

  double distance(const Position2D &other) const noexcept;
  double direction(const Position2D &other) const noexcept;
  double angular_error(const Position2D &other) const noexcept;
};

} // namespace citylab

#endif // CITYLAB_UTILS_H