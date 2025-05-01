#include "robot_patrol/naive_goto.hpp"
#include "rclcpp/logging.hpp"
#include "robot_patrol/utils.hpp"

namespace citylab {

void NaiveGoto::update_position(const Position2D &position) {
  position_ = position;
}

void NaiveGoto::set_target(const Position2D &target) {
  target_ = target;
  state_ = DIRECTION;
  RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"), "Set state to DIRECTION");
  angular_pid_.reset();
  linear_pid_.reset();
}

bool NaiveGoto::target_reached() const { return state_ == DONE; }

Position2D NaiveGoto::get_position() const { return position_; }

Position2D NaiveGoto::get_cmd_vel() {
  Position2D cmd_vel;

  float direction_error = position_.direction(target_) - position_.theta;
  float distance_error = position_.distance(target_);
  float orientation_error = position_.angular_error(target_);

  // Update state
  switch (state_) {
  case DONE:
    return cmd_vel;
  case DIRECTION:
    if (abs(direction_error) < EPSILON) {
      state_ = DISTANCE;
      angular_pid_.reset();
      RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"), "Set state to DISTANCE");
    }
    break;
  case DISTANCE:
    if (abs(distance_error) < EPSILON) {
      state_ = ORIENTATION;
      angular_pid_.reset();
      linear_pid_.reset();
      RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"), "Set state to ORIENTATION");
    }
    break;
  case ORIENTATION:
    if (abs(orientation_error) < EPSILON) {
      state_ = DONE;
      angular_pid_.reset();
      RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"), "Set state to DONE");
    }
    break;
  default:
    state_ = DONE; // Something went wrong
  }

  // Set velocity
  switch (state_) {
  case DIRECTION:
    cmd_vel.theta = angular_pid_.step(direction_error);
    break;
  case DISTANCE:
    cmd_vel.theta = angular_pid_.step(direction_error);
    cmd_vel.x = linear_pid_.step(distance_error);
    break;
  case ORIENTATION:
    cmd_vel.theta = angular_pid_.step(orientation_error);
    break;
  default:
    state_ = DONE; // Something went wrong
  }

  RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"),
              "dir=%.3f dist=%.3f or=%.3f x=%.3f th=%.3f", direction_error,
              distance_error, orientation_error, cmd_vel.x, cmd_vel.theta);

  return cmd_vel;
}
void NaiveGoto::set_linear_pid(const SimplePID &pid) { linear_pid_ = pid; }
void NaiveGoto::set_angular_pid(const SimplePID &pid) { angular_pid_ = pid; }

} // namespace citylab