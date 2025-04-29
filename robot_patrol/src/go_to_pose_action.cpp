#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/naive_goto.hpp"
#include "robot_patrol/utils.hpp"
#include "robot_patrol_msgs/action/go_to_pose.hpp"

using GoToPose = robot_patrol_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using namespace citylab;

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

  NaiveGoto naive_goto_;

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
      this, "go_to_pose",
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
  naive_goto_.update_position(Position2D::from_odometry(*msg));
}

rclcpp_action::GoalResponse
GoToActionServer::goal_handler_(const rclcpp_action::GoalUUID &uuid,
                                std::shared_ptr<const GoToPose::Goal> goal) {
  Position2D target = Position2D::from_pose2D(goal->goal_pos);
  RCLCPP_INFO(this->get_logger(), "Received goal location %s",
              target.to_str().c_str());
  naive_goto_.set_target(target);
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

  while (!naive_goto_.target_reached()) {
    if (goal_handle->is_canceling()) {
      result->status = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    velocity_pub_->publish(naive_goto_.get_cmd_vel().to_Twist());
    feedback->current_pos = naive_goto_.get_position().to_Pose2D();
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