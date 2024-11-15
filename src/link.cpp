#include "link.hpp"


Link::Link(const size_t index, const Eigen::Vector3d mean, const Eigen::Matrix3d cov)
  : mInitialMean{mean}, mInitialCov{cov}, mMean{mean}, mCov{cov}, mIndex{index}
{
  mName = std::string("link") + std::to_string(index);
}

Link::~Link()
{
}

void Link::updateParameters(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation, const Eigen::MatrixXd J, const Eigen::MatrixXd localRotation)
{
  Eigen::Vector3d updatedMean = translation + rotation * mInitialMean;
  mMean = updatedMean;
  Eigen::MatrixXd updatedCov = rotation * mInitialCov * rotation.transpose();
  mCov = updatedCov;
  Eigen::Vector3d rotationAxis = J(Eigen::seq(3, Eigen::indexing::last), getIndex()-1);
  Eigen::MatrixXd rotationLocalDerivate = derivateLocalRotation(rotationAxis, localRotation);
}


Eigen::MatrixXd Link::derivateLocalRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation) {
  Eigen::MatrixXd skewedRotationAxis (rotation.rows(), rotation.cols());
  skewedRotationAxis << 0, -rotationAxis(2), rotationAxis(1),
                rotationAxis(2), 0, -rotationAxis(0),
                -rotationAxis(1), rotationAxis(0), 0;
  return skewedRotationAxis * rotation;
}


void Link::printParameters()
{
  std::cout << "\u03BC = \n" << mMean << '\n';
  std::cout << "\u03A3 = \n" << mCov << std::endl;
}
