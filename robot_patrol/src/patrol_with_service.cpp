#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/simple_lidar.hpp"
#include "robot_patrol_msgs/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace citylab;
using GetDirection = robot_patrol_msgs::srv::GetDirection;

float degree_to_radian(float degree) { return degree * 0.0174532925f; }

//============================================================================================
class DirectionClient {
public:
  enum Status {
    NOT_INITIALIZED,
    INITIALIZED,
    IN_PROGRESS,
    DONE,
    NOT_AVAILABLE,
    ERROR
  };
  enum Direction { LEFT, CENTER, RIGHT, UNKNOWN };

  DirectionClient(double timeout_s = 0.0);
  ~DirectionClient() = default;
  DirectionClient(const DirectionClient &) = delete;

  void init(rclcpp::Node *parent_node, std::string topic) noexcept;
  bool send_request(sensor_msgs::msg::LaserScan::SharedPtr msg) noexcept;
  Status get_status() const noexcept;
  Direction get_direction() const noexcept;
  bool ready() const noexcept;

private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  double timeout_s_;
  Status status_ = NOT_INITIALIZED;
  Direction direction_ = UNKNOWN;

  void response_callback_(rclcpp::Client<GetDirection>::SharedFuture future);
};

DirectionClient::DirectionClient(double timeout_s) {
  if (timeout_s < 0.0) {
    throw std::invalid_argument(
        "DirectionClient - Attempted to set timeout value to less than 0");
  }
  timeout_s_ = timeout_s;
}

void DirectionClient::init(rclcpp::Node *parent_node,
                           std::string topic) noexcept {
  client_ = parent_node->create_client<GetDirection>(topic);
  if (!client_) {
    status_ = ERROR;
    return;
  }
  status_ = INITIALIZED;
}

bool DirectionClient::send_request(
    sensor_msgs::msg::LaserScan::SharedPtr msg) noexcept {
  try {
    if (!this->ready()) {
      return false;
    }

    if (!msg) {
      return false;
    }

    if (timeout_s_ == 0 && !client_->service_is_ready()) {
      status_ = NOT_AVAILABLE;
      return false;
    } else {
      auto timeout = std::chrono::duration<double>(timeout_s_);
      if (!client_->wait_for_service(timeout)) {
        status_ = NOT_AVAILABLE;
        return false;
      }
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;
    auto result = client_->async_send_request(
        request, std::bind(&DirectionClient::response_callback_, this,
                           std::placeholders::_1));
    status_ = IN_PROGRESS;
    return true;
  } catch (...) {
    // Under no circumstances DirectionClient should throw exceptions.
    // Any issues should be handled by switching status to ERROR.
    status_ = ERROR;
    return false;
  }
}

void DirectionClient::response_callback_(
    rclcpp::Client<GetDirection>::SharedFuture future) {
  auto response = future.get();
  status_ = DONE;
  if (response->direction == "left") {
    direction_ = LEFT;
  } else if (response->direction == "right") {
    direction_ = RIGHT;
  } else if (response->direction == "center") {
    direction_ = CENTER;
  } else {
    direction_ = UNKNOWN;
    status_ = ERROR;
  }
}

DirectionClient::Status DirectionClient::get_status() const noexcept {
  return status_;
}

DirectionClient::Direction DirectionClient::get_direction() const noexcept {
  return direction_;
}

bool DirectionClient::ready() const noexcept {
  return status_ == INITIALIZED || status_ == DONE;
}

//============================================================================================
class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol() = default;

private:
  constexpr static const float DETECTION_DISTANCE_ = 0.35f;
  constexpr static const double TIMER_FREQUENCY_ = 10.0;
  constexpr static const int FRONT_HALF_WIDTH_ = 40;

  bool obstacle_in_front_ = false;

  SimpleLidar lidar_;

  DirectionClient direction_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

  void calculate_and_publish_velocity_();
  void laser_callback_(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void stop_();
};

Patrol::Patrol() : Node("patrol_with_service") {
  using std::placeholders::_1;
  timer_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  laser_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = laser_callback_group_;
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_callback_, this, _1), sub_options);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::calculate_and_publish_velocity_, this),
      timer_callback_group_);

  direction_client_.init(this, "direction_service");

  rclcpp::on_shutdown([this]() { this->stop_(); });
}

void Patrol::calculate_and_publish_velocity_() {
  geometry_msgs::msg::Twist velocity;
  if (!obstacle_in_front_) {
    velocity.linear.x = 0.1;
    velocity.angular.z = 0.0;
  } else if (direction_client_.get_status() == DirectionClient::DONE) {
    velocity.linear.x = 0.1;
    switch (direction_client_.get_direction()) {
    case DirectionClient::CENTER:
      velocity.angular.z = 0.0;
      break;
    case DirectionClient::LEFT:
      velocity.angular.z = 0.5;
      break;
    case DirectionClient::RIGHT:
      velocity.angular.z = 0.5;
      break;
    default:
      velocity.linear.x = 0.0;
      velocity.angular.z = 0.0;
      RCLCPP_WARN(this->get_logger(), "DirectionClient direction is unknown.");
    }
  } else {
    if (direction_client_.get_status() == DirectionClient::IN_PROGRESS) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for direction service response.");
    }
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
  }
  velocity_pub_->publish(velocity);
  // RCLCPP_INFO(this->get_logger(), "UPDADING VELOCITY %f",
  // velocity.angular.z);
}

void Patrol::laser_callback_(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  lidar_.update(msg);
  LidarMeasurement front = lidar_.get_closest_range(0, degree_to_radian(60));
  RCLCPP_INFO(this->get_logger(), "Front - %s", front.str().c_str());
  if (front.state != LidarMeasurement::OK) {
    RCLCPP_WARN(this->get_logger(),
                "Unable to get proper readout from front of lidar");
    obstacle_in_front_ = true;
  }

  if (front.distance > DETECTION_DISTANCE_) {
    RCLCPP_INFO(this->get_logger(),
                "No obstacle in front, setting direction to 0.0.");
    obstacle_in_front_ = false;
  }

  if (obstacle_in_front_ && direction_client_.ready()) {
    direction_client_.send_request(msg);
  }
}

void Patrol::stop_() {
  RCLCPP_INFO(this->get_logger(), "Shutting down - Sending stop signal");

  geometry_msgs::msg::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.angular.z = 0.0;

  velocity_pub_->publish(stop_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol_node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  rclcpp::shutdown();
  return 0;
}