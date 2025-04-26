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

  static std::vector<std::string> responses = {
      std::string("right"),
      std::string("center"),
      std::string("left"),
  };
  int responses_size = static_cast<int>(responses.size());
  float angle_60 = SimpleLidar::degree_to_radian(60);
  int max_idx = -1;
  float max_distance = -1.0f;
  for (int i = 0; i < responses_size; ++i) {
    LidarMeasurement measurement = lidar.get_sum(angle_60 * (i - 1), angle_60);
    if (measurement.state != LidarMeasurement::OK)
      continue;
    if (measurement.distance > max_distance) {
      max_distance = measurement.distance;
      max_idx = i;
    }
  }

  if (max_idx == -1) {
    RCLCPP_WARN(this->get_logger(),
                "Unable to get proper readout, returning direction \"%s\"",
                responses[1].c_str());
    response->direction = responses[1];
  } else {
    response->direction = responses[max_idx];
  }
}

//--------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionServiceNode>());
  rclcpp::shutdown();
  return 0;
}