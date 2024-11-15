#include "robot.hpp"



Robot::Robot(const int numberOfJoints, const std::string urdfFile) 
  : mNumberOfJoints{numberOfJoints}
{
  pinocchio::urdf::buildModel(urdfFile, mModel);
  pinocchio::Data data(mModel);
  mData = data;


  const Eigen::Vector3d mean0(0, 0.5, 0);
  Eigen::Matrix3d cov0(3, 3);
  cov0(0,0) = 0.02;
  cov0(1,0) = -0.0004;
  cov0(2,0) = -0.0002;
  cov0(0,1) = cov0(1,0);
  cov0(1,1) = 0.0839;
  cov0(2,1) = 0.001;
  cov0(0,2) = cov0(2,0);
  cov0(1,2) = cov0(2,1);
  cov0(2,2) = 0.021;

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

  std::vector<Eigen::MatrixXd> rotations(mModel.nv);

  for (Link& link: mLinks)
  {
    size_t frameId = mModel.getFrameId(link.getName());
    Eigen::Vector3d translation = mData.oMf[frameId].translation();
    Eigen::Matrix3d rotation = mData.oMf[frameId].rotation();
    rotations[link.getIndex()-1] = rotation;
    Eigen::MatrixXd J(6, mModel.nv);
    pinocchio::computeFrameJacobian(mModel, mData, jointConfiguration, frameId, J);
    // first order derivation of rotation
    Eigen::Vector3d rotationAxis = J(Eigen::seq(3, Eigen::indexing::last), link.getIndex()-1);
    Eigen::MatrixXd localRotation = (link.getIndex()==1) ? rotation : rotation * rotations[0].inverse();
    link.updateParameters(translation, rotation, J, localRotation);

  }

}



