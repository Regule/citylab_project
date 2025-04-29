#ifndef CITYLAB_NAIVE_GOTO
#define CITYLAB_NAIVE_GOTO

#include "robot_patrol/utils.hpp"

namespace citylab {

class NaiveGoto {
public:
  NaiveGoto() = default;
  ~NaiveGoto() = default;
  NaiveGoto(const NaiveGoto &) = delete;

  void update_position(const Position2D &position);
  void set_target(const Position2D &target);
  bool target_reached() const;
  Position2D get_cmd_vel();
  Position2D get_position() const;
  void set_linear_pid(const SimplePID &pid);
  void set_angular_pid(const SimplePID &pid);

private:
  enum State { DIRECTION, DISTANCE, ORIENTATION, DONE };
  constexpr static const float EPSILON = 0.01;

  Position2D target_ = Position2D(0.0, 0.0, 0.0);
  Position2D position_ = Position2D(0.0, 0.0, 0.0);
  State state_ = DONE;
  SimplePID linear_pid_;
  SimplePID angular_pid_;
};

} // namespace citylab

#endif // CITYLAB_NAIVE_GOTO