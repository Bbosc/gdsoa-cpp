#include "Robot.hpp"



Robot::Robot(const int numberOfJoints, const std::string urdfFile) 
  : mNumberOfJoints{numberOfJoints}
{
  pinocchio::urdf::buildModel(urdfFile, mModel);
  pinocchio::Data data(mModel);
  mData = data;


  const Eigen::Vector2d mean0(0, 0.5);
  Eigen::MatrixXd cov0(2, 2);
  cov0(0,0) = 0.1;
  cov0(1,0) = 0;
  cov0(0,1) = 0;
  cov0(1,1) = 0.1;

  for (size_t i = 0; i < mNumberOfJoints; i++)
  {
    Eigen::Vector2d offset(0, i);
    Link link(mean0 + offset, cov0);
    mLinks.push_back(link);
  }
  
}

Robot::~Robot() 
{
}

void Robot::displayLinks()
{
  for (size_t i {0}; i < mNumberOfJoints; i++)
  {
    std::cout << "Link " << i << ": ";
    mLinks[i].printParameters();
  }
}

void Robot::move(const Eigen::VectorXd jointConfiguration)
{
  pinocchio::forwardKinematics(mModel, mData, jointConfiguration);
  std::cout << mData.oMi[2].translation().transpose() << std::endl;

}
