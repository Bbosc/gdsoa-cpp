#include <math.h>
#include "robot.hpp"
#include "distribution.hpp"


int main (int argc, char *argv[]) {

  double minAngle {-M_PI/2};
  double maxAngle {M_PI/2};
  size_t resolution {10};
  std::vector<double> angles {linspace(minAngle, maxAngle, resolution)};

  Robot robot(2, std::string("../description/2_joints_arm.urdf"));

  Eigen::Vector3d obstaclePosition(0.1, 0.5, 0);
  std::vector<Eigen::Vector3d> probabilityMap;

  for (size_t i{0}; i < angles.size(); i++)
  {
    for (size_t j{0}; j < angles.size(); j++)
    {   
      Eigen::Vector2d q(angles[i], angles[j]);
      robot.move(q);
      double collisionProbability {0};
      for (size_t k{0}; k < robot.getLinks().size(); k++)
      {
        collisionProbability += computeMultivariateDistribution(
          obstaclePosition, robot.getLinks()[k].getMean(), robot.getLinks()[k].getCov()); 
      }
      std::cout << collisionProbability << '\n';
      Eigen::Vector3d location (angles[i], angles[j], collisionProbability);
      probabilityMap.push_back(location);

    }
  }

  return 0;
}
