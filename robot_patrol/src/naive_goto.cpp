#include "robot_patrol/naive_goto.hpp"

namespace citylab {

void NaiveGoto::update_position(const Position2D &position) {
  position_ = position;
}

void NaiveGoto::set_target(const Position2D &target) {
  target_ = target;
  state_ = DIRECTION;
}

bool NaiveGoto::target_reached() const {
  return (position_.distance(target_) < EPSILON &&
          position_.angular_error(target_) < EPSILON);
}

Position2D NaiveGoto::get_position() const { return position_; }

Position2D NaiveGoto::get_cmd_vel() {
  Position2D cmd_vel;

  float direction_error = position_.direction(target_) - position_.theta;
  float distance_error = position_.distance(target_);
  float orientation_error = position_.angular_error(target_);

  if (state_ == DIRECTION) {
    if (abs(direction_error) <= EPSILON) {
      state_ = DISTANCE;
    } else {
      cmd_vel.theta = 0.8 * direction_error;
      if (abs(cmd_vel.theta) < 0.05) {
        cmd_vel.theta = 0.1 * direction_error / abs(direction_error);
      }
    }
  }

  if (state_ == DISTANCE) {
    if (abs(distance_error) <= EPSILON) {
      state_ = ORIENTATION;
    } else {
      cmd_vel.x = 0.1;
    }
  }

  if (state_ == ORIENTATION) {
    if (abs(orientation_error) <= EPSILON) {
      state_ = DONE;
    } else {
      cmd_vel.theta = 0.2 * orientation_error;
      if (abs(cmd_vel.theta) < 0.1) {
        cmd_vel.theta = 0.1 * orientation_error / abs(orientation_error);
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("goto_ctrl"),
              "dir=%.3f dist=%.3f or=%.3f x=%.3f th=%.3f", direction_error,
              distance_error, orientation_error, cmd_vel.x, cmd_vel.theta);

  return cmd_vel;
}

} // namespace citylab