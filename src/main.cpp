#include <math.h>
#include "robot.hpp"
#include "distribution.hpp"


int main (int argc, char *argv[]) {
  Eigen::Vector2d q(M_PI/4, -M_PI/4);
  Robot robot(2, std::string("../description/2_joints_arm.urdf"));
  robot.move(q);

  std::cout << "computing collision proba for obstacle: " ;
  Eigen::Vector3d x(0.1, 0.5, 0);
  double p = computeMultivariateDistribution(
    x, robot.getLinks()[0].getMean(), robot.getLinks()[0].getCov()); 
  std::cout << p << std::endl;
  return 0;
}
