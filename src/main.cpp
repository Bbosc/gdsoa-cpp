#include <math.h>
#include "Robot.hpp"


int main (int argc, char *argv[]) {
  Eigen::Vector2d q(M_PI/4, -M_PI/4);
  Robot robot(2, std::string("../description/2_joints_arm.urdf"));
  robot.move(q);
  return 0;
}
