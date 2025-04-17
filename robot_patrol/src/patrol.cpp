#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>

float degree_to_radian(float degree) { return degree * 0.0174532925f; }

//============================================================================================

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

class LidarAwareness {
public:
  LidarAwareness() = default;
  explicit LidarAwareness(const LidarConfig &cfg);
  ~LidarAwareness() = default;

  void update(sensor_msgs::msg::LaserScan::SharedPtr msg);

  LidarMeasurement get_closest_range(float angle, float cone_size) const;
  LidarMeasurement get_farthest_range(float angle, float cone_size) const;

private:
  std::unique_ptr<LidarConfig> cfg_;
  std::vector<float> scan_;

  void build_config_(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

//-------------------------
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

LidarAwareness::LidarAwareness(const LidarConfig &cfg) {
  cfg_ = std::make_unique<LidarConfig>(cfg);
}

void LidarAwareness::update(sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

void LidarAwareness::build_config_(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  cfg_ = std::make_unique<LidarConfig>();
  cfg_->angle_min = msg->angle_min;
  cfg_->step = msg->angle_increment;
  cfg_->sample_count = msg->ranges.size();
  cfg_->range.first = msg->range_min;
  cfg_->range.second = msg->range_max;
}

LidarMeasurement LidarAwareness::get_closest_range(float angle,
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

LidarMeasurement LidarAwareness::get_farthest_range(float angle,
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
//============================================================================================
class Patrol : public rclcpp::Node {
public:
  Patrol();
  ~Patrol() = default;

private:
  constexpr static const float DETECTION_DISTANCE_ = 0.35f;
  constexpr static const double TIMER_FREQUENCY_ = 10.0;
  constexpr static const int FRONT_HALF_WIDTH_ = 40;

  float direction_ = 0.0f;
  float velocity_ = 0.1f;
  LidarAwareness lidar_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

  void calculate_and_publish_velocity_();
  void laser_callback_(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void stop_();
};

Patrol::Patrol() : Node("patrol") {
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

  rclcpp::on_shutdown([this]() { this->stop_(); });
}

void Patrol::calculate_and_publish_velocity_() {
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = velocity_;
  velocity.angular.z = direction_ / 2.0f;
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
    velocity_ = 0.0f;
  }

  velocity_ = 0.1f;
  if (front.distance > DETECTION_DISTANCE_) {
    RCLCPP_INFO(this->get_logger(),
                "No obstacle in front, setting direction to 0.0.");
    direction_ = 0.0f;
    return;
  }

  LidarMeasurement closest =
      lidar_.get_farthest_range(0, degree_to_radian(180));
  RCLCPP_INFO(this->get_logger(),
              "Obstacle in front detected, setting direction to %f.",
              closest.angle);
  direction_ = closest.angle;
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
  executor.spin();
  rclcpp::shutdown();
  return 0;
}