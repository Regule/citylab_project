#include "robot_patrol/utils.hpp"

namespace citylab {

geometry_msgs::msg::Twist Position2D::to_Twist() const {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = x;
  twist.angular.z = theta;
  if (y != 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("Position2D"),
                "Attempted to set y to nonzero value. Ignoring.");
  }
  return twist;
}

geometry_msgs::msg::Pose2D Position2D::to_Pose2D() const {
  auto pose = geometry_msgs::msg::Pose2D();
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

Position2D Position2D::from_odometry(const nav_msgs::msg::Odometry &msg) {
  Position2D position{msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.orientation.z};
  return position;
}

Position2D Position2D::from_pose2D(const geometry_msgs::msg::Pose2D &msg) {
  Position2D position{msg.x, msg.y, msg.theta};
  return position;
}

double Position2D::distance(const Position2D &other) const noexcept {
  float dx = this->x - other.x;
  float dy = this->y - other.y;
  return std::sqrt(dx * dx + dy * dy);
}

double Position2D::direction(const Position2D &other) const noexcept {
  return atan2(other.y - this->y, other.x - this->x);
}

double Position2D::angular_error(const Position2D &other) const noexcept {
  return other.theta - this->theta;
}

std::string Position2D::to_str() const {
  std::stringstream descritpion;
  descritpion << std::setprecision(3) << x << " " << std::setprecision(3) << y;
  descritpion << " " << std::setprecision(3) << theta;
  std::string descritpion_str = descritpion.str();
  return descritpion_str;
}

} // namespace citylab