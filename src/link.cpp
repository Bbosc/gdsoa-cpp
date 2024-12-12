#include "link.hpp"


Link::Link(const size_t index, const Mean mu, const Cov sigma)
  : mInitialMean{mu}, mInitialCov{sigma}, mMean{mu}, mCov{sigma}, mIndex{index}
{
  mName = std::string("link") + std::to_string(index);
}

Link::~Link()
{
}

void Link::updateParameters(const Tra T, const Rot R)
{
	Eigen::Vector3d updatedMean = T + R * mInitialMean;
	mMean = updatedMean;
	Eigen::MatrixXd updatedCov = R * mInitialCov * R.transpose();
	mCov = updatedCov;
	//Eigen::Vector3d rotationAxis = J(Eigen::seq(3, Eigen::indexing::last), getIndex()-1);
	//Eigen::MatrixXd relativeRotationDerivate = derivateRelativeRotation(rotationAxis, localRotation);
}

Eigen::MatrixXd derivateMu(const Eigen::MatrixXd dR, const Mean mu, const Eigen::MatrixXd dT)
{
	return dT + dR * mu;
}

Eigen::MatrixXd derivateSigma(const Rot R, const Eigen::MatrixXd dR, const Cov sigma)
{
	return dR * sigma * R + R * sigma * dR;
}


Eigen::MatrixXd Link::derivateRelativeRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation) {
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
