#include "Robot.hpp"


Robot::Robot(const int numberOfJoints) 
  : mNumberOfJoints{numberOfJoints}
{
  const Eigen::Vector2d mean0(0, 0.5);
  Eigen::MatrixXd cov0(2, 2);
  cov0(0,0) = 0.1;
  cov0(1,0) = 0;
  cov0(0,1) = 0;
  cov0(1,1) = 0.1;

  for (size_t i = 0; i < mNumberOfJoints; i++)
  {
    std::cout << "adding link to robot\n";
    Link link(mean0, cov0);
    mLinks.push_back(link);
  }
  
}

Robot::~Robot() 
{
  std::cout << "destroying robot" << std::endl;
}

