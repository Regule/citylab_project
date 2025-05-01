#include "robot_patrol/utils.hpp"
#include <cmath>
#include <iostream>

using namespace citylab;

int main(int argc, char **argv) {
  Position2D odometry;
  Position2D target;

  target.theta = M_PI / 2.0;

  for (odometry.theta = 0.0; odometry.theta < 2 * M_PI;
       odometry.theta += 0.01) {
    std::cout << odometry.theta << ";" << odometry.angular_error(target)
              << std::endl;
  }
}