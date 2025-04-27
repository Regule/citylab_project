#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol_msgs/action/go_to_pose.hpp"

using GoToPose = robot_patrol_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

//-------------------------------------------------------------------------------------------
//                                        POSITION2D
//-------------------------------------------------------------------------------------------
struct Position2D {
  double x;
  double y;
  double theta;

  double distance(const Position2D &other) const noexcept;
  double direction(const Position2D &other) const noexcept;
  double angular_error(const Position2D &other) const noexcept;

  geometry_msgs::msg::Pose2D to_Pose2D() const;
  std::string to_str() const;

  static Position2D from_odometry(const nav_msgs::msg::Odometry &msg);
  static Position2D from_pose2D(const geometry_msgs::msg::Pose2D &msg);
};

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

//-------------------------------------------------------------------------------------------
//                                   GOTO CONTROLLER
//-------------------------------------------------------------------------------------------

class GoToController {
public:
  GoToController() = default;
  ~GoToController() = default;
  GoToController(const GoToController &) = delete;

  void update_position(const Position2D &position);
  void set_target(const Position2D &target);
  bool target_reached() const;
  geometry_msgs::msg::Twist get_cmd_vel() const;
  Position2D get_position() const;

private:
  constexpr static const float EPSILON = 0.001;

  Position2D target_{0.0, 0.0, 0.0};
  Position2D position_{0.0, 0.0, 0.0};
};

void GoToController::update_position(const Position2D &position) {
  position_ = position;
}

void GoToController::set_target(const Position2D &target) { target_ = target; }

bool GoToController::target_reached() const {
  return (position_.distance(target_) < EPSILON &&
          position_.angular_error(target_) < EPSILON);
}

Position2D GoToController::get_position() const { return position_; }

geometry_msgs::msg::Twist GoToController::get_cmd_vel() const {
  auto cmd_vel = geometry_msgs::msg::Twist();

  float direction_error = position_.direction(target_);
  if (direction_error > EPSILON) {
    cmd_vel.angular.z = 0.5 * direction_error;
    return cmd_vel;
  }

  float distance_error = position_.distance(target_);
  if (distance_error > EPSILON) {
    cmd_vel.linear.x = 0.3;
    return cmd_vel;
  }

  float orientation_error = position_.angular_error(target_);
  if (orientation_error > EPSILON) {
    cmd_vel.angular.z = 0.2 * orientation_error;
    return cmd_vel;
  }
  return cmd_vel; // Stay in place
}

//-------------------------------------------------------------------------------------------
//                                   DISTANCE ACTION SERVER
//-------------------------------------------------------------------------------------------
class GoToActionServer : public rclcpp::Node {
public:
  explicit GoToActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;

  GoToController go_to_ctrl_;

  void update_odometry_(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse
  goal_handler_(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const GoToPose::Goal> goal);
  rclcpp_action::CancelResponse
  cancel_handler_(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void accepted_handler_(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
  void execute_(const std::shared_ptr<GoalHandleGoToPose> goal_handle);
};

GoToActionServer::GoToActionServer(const rclcpp::NodeOptions &options)
    : Node("distance_action_server", options) {
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<GoToPose>(
      this, "distance_as",
      std::bind(&GoToActionServer::goal_handler_, this, _1, _2),
      std::bind(&GoToActionServer::cancel_handler_, this, _1),
      std::bind(&GoToActionServer::accepted_handler_, this, _1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToActionServer::update_odometry_, this, _1));
  velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void GoToActionServer::update_odometry_(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  go_to_ctrl_.update_position(Position2D::from_odometry(*msg));
}

rclcpp_action::GoalResponse
GoToActionServer::goal_handler_(const rclcpp_action::GoalUUID &uuid,
                                std::shared_ptr<const GoToPose::Goal> goal) {
  Position2D target = Position2D::from_pose2D(goal->goal_pos);
  RCLCPP_INFO(this->get_logger(), "Received goal location %s",
              target.to_str().c_str());
  go_to_ctrl_.set_target(target);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GoToActionServer::cancel_handler_(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToActionServer::accepted_handler_(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  using namespace std::placeholders;

  std::thread{std::bind(&GoToActionServer::execute_, this, _1), goal_handle}
      .detach();
}

void GoToActionServer::execute_(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GoToPose::Feedback>();
  auto result = std::make_shared<GoToPose::Result>();
  rclcpp::Rate loop_rate(1);

  while (!go_to_ctrl_.target_reached()) {
    if (goal_handle->is_canceling()) {
      result->status = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    velocity_pub_->publish(go_to_ctrl_.get_cmd_vel());
    feedback->current_pos = go_to_ctrl_.get_position().to_Pose2D();
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    // No need for lock as we read single value
    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

//-------------------------------------------------------------------------------------------
//                                       MAIN
//-------------------------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto distance_server = std::make_shared<GoToActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(distance_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}