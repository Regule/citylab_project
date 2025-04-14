#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol() = default;

private:
  constexpr static const float DETECTION_DISTANCE_ = 0.35f;
  constexpr static const double TIMER_FREQUENCY_ = 10.0;

  float direction_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  void calculate_and_publish_velocity_();
  void laser_callback_(sensor_msgs::msg::LaserScan::SharedPtr msg);

  static bool is_forward_path_open_(sensor_msgs::msg::LaserScan::SharedPtr msg);
  static float
  get_angle_with_largest_distance_(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

Patrol::Patrol() : Node("patrol") {
  using std::placeholders::_1;
  velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_callback_, this, _1));
  auto timer_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / TIMER_FREQUENCY_));
  timer_ = this->create_wall_timer(
      timer_period, std::bind(&Patrol::calculate_and_publish_velocity_, this));
}

void Patrol::calculate_and_publish_velocity_() {
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.1;
  velocity.angular.z = direction_ / 2;
  velocity_pub_->publish(velocity);
}

void Patrol::laser_callback_(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  direction_ =
      is_forward_path_open_(msg) ? 0.0f : get_angle_with_largest_distance_(msg);
}

bool Patrol::is_forward_path_open_(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  for (int i = 0; i < 20; ++i) {
    if (msg->ranges[i] <= DETECTION_DISTANCE_) {
      return false;
    }
  }
  for (int i = 360 - 1; i >= 360 - 1 - 20; --i) {
    if (msg->ranges[i] <= DETECTION_DISTANCE_) {
      return false;
    }
  }
  return true;
}

float Patrol::get_angle_with_largest_distance_(
    sensor_msgs::msg::LaserScan::SharedPtr msg) {
  float largest_distance = 0.0f;
  int scan_index = 0;
  for (int i = 0; i < 180; ++i) {
    if (!std::isinf(msg->ranges[i]) && msg->ranges[i] > largest_distance) {
      largest_distance = msg->ranges[i];
      scan_index = i;
    }
  }
  for (int i = 360 - 1; i >= 360 - 1 - 180; --i) {
    if (!std::isinf(msg->ranges[i]) && msg->ranges[i] > largest_distance) {
      largest_distance = msg->ranges[i];
      scan_index = i - 360;
    }
  }
  return scan_index * 3.141 / 180;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}