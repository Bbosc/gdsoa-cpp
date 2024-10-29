#include "Robot.hpp"



Robot::Robot(const int numberOfJoints, const std::string urdfFile) 
  : mNumberOfJoints{numberOfJoints}
{
  pinocchio::urdf::buildModel(urdfFile, mModel);
  pinocchio::Data data(mModel);
  mData = data;


  const Eigen::Vector3d mean0(0, 0.5, 0);
  Eigen::Matrix3d cov0(3, 3);
  cov0(0,0) = 0.1;
  cov0(1,0) = 0.2;
  cov0(2,0) = 0.3;
  cov0(0,1) = 0;
  cov0(1,1) = 0.1;
  cov0(2,1) = 0.0;
  cov0(0,2) = 0.4;
  cov0(1,2) = 0.0;
  cov0(2,2) = 0.1;

  for (size_t i = 0; i < mNumberOfJoints; i++)
  {
    Link link(i+1, mean0, cov0);
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
    std::cout << "Link " << i << ":\n";
    mLinks[i].printParameters();
  }
}

void Robot::move(const Eigen::VectorXd jointConfiguration)
{
  pinocchio::forwardKinematics(mModel, mData, jointConfiguration);
  pinocchio::updateFramePlacements(mModel, mData);

  for (auto& link: mLinks)
  {
    size_t frameId = mModel.getFrameId(link.getName());
    Eigen::Vector3d translation = mData.oMf[frameId].translation();
    Eigen::Matrix3d rotation = mData.oMf[frameId].rotation();
    link.updateParameters(translation, rotation);
  }

  displayLinks();


}

