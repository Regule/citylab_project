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

private:
  enum State { DIRECTION, DISTANCE, ORIENTATION, DONE };
  constexpr static const float EPSILON = 0.001;

  Position2D target_{0.0, 0.0, 0.0};
  Position2D position_{0.0, 0.0, 0.0};
  State state_ = DONE;
};

} // namespace citylab

#endif // CITYLAB_NAIVE_GOTO