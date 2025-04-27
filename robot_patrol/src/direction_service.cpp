#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

#include "robot_patrol/simple_lidar.hpp"
#include "robot_patrol_msgs/srv/get_direction.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

using LaserScan = sensor_msgs::msg::LaserScan;
using GetDirection = robot_patrol_msgs::srv::GetDirection;

using namespace citylab;

//------------------------------------------------------------------------------------------
class DirectionServiceNode : public rclcpp::Node {
public:
  DirectionServiceNode();

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void spin_callback_(const std::shared_ptr<GetDirection::Request> request,
                      const std::shared_ptr<GetDirection::Response> response);
};

DirectionServiceNode::DirectionServiceNode() : Node("direction_service") {
  using namespace std::placeholders;
  srv_ = create_service<GetDirection>(
      "direction_service",
      std::bind(&DirectionServiceNode::spin_callback_, this, _1, _2));
}

void DirectionServiceNode::spin_callback_(
    const std::shared_ptr<GetDirection::Request> request,
    const std::shared_ptr<GetDirection::Response> response) {
  LaserScan::SharedPtr laser = std::make_shared<LaserScan>(request->laser_data);
  SimpleLidar lidar;
  lidar.update(laser);

  float angle_60 = SimpleLidar::degree_to_radian(60);

  LidarMeasurement left = lidar.get_sum(angle_60, angle_60);
  LidarMeasurement center = lidar.get_sum(0.0f, angle_60);
  LidarMeasurement right = lidar.get_sum(-angle_60, angle_60);
  if (left.state != LidarMeasurement::OK &&
      center.state != LidarMeasurement::OK &&
      right.state != LidarMeasurement::OK) {
    RCLCPP_WARN(this->get_logger(),
                "No valid measurement found, setting direction right");
    response->direction = "right";
    return;
  }

  if (left.state != LidarMeasurement::OK)
    left.distance = -1;
  if (right.state != LidarMeasurement::OK)
    right.distance = -1;
  if (center.state != LidarMeasurement::OK)
    center.distance = -1;

  RCLCPP_INFO(this->get_logger(), "Distance sums <%0.3f %0.3f %0.3f>",
              left.distance, center.distance, right.distance);

  if (left.distance > center.distance && left.distance > right.distance) {
    response->direction = "left";
    return;
  }
  if (center.distance > left.distance && center.distance > right.distance) {
    response->direction = "forward";
    return;
  }
  response->direction = "right";
}

//--------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionServiceNode>());
  rclcpp::shutdown();
  return 0;
}