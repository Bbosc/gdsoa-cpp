#include "link.hpp"


Link::Link(const Mean mu, const Cov sigma, const size_t index)
  : initialMean{mu}, initialCov{sigma}, mean{mu}, cov{sigma}
{
  name = std::string("link") + std::to_string(index);
}

Link::~Link()
{
}

void Link::updateParameters(const Tra T, const Rot R, const Eigen::MatrixXd J)
{
	mean = T + R * initialMean;
	cov = R * initialCov * R.transpose();
	this->R = R;
	this->T = T;
	dT = J.topRows(3);
	p = dist::computeMultivariateDistribution(mean, cov);
	//Eigen::Vector3d rotationAxis = J(Eigen::seq(3, Eigen::indexing::last), getIndex()-1);
	//Eigen::MatrixXd relativeRotationDerivate = derivateRelativeRotation(rotationAxis, localRotation);
}

Eigen::MatrixXd Link::deriveCollision(const Eigen::MatrixXd dR, const Eigen::MatrixXd dT) {
	Eigen::Vector3d diff = dist::obstaclePosition - mean;
	Eigen::Vector3d dpdmu = dist::deriveCollisionMu(p, cov, diff);
	Eigen::Matrix3d dpdsigma = dist::deriveCollisionSigma(p, cov, diff);
	Eigen::Vector3d dmudq = deriveMu(dR, dT);
	Eigen::Matrix3d dsigmadq = deriveSigma(dR);
	return dpdmu * dmudq.transpose() + dpdsigma * dsigmadq;
}

Eigen::Vector3d Link::deriveMu(const Eigen::MatrixXd dR, const Eigen::MatrixXd dT)
{
	return dT + dR * mean;
}

Eigen::Matrix3d Link::deriveSigma(const Eigen::MatrixXd dR)
{
	return dR * cov * R + R * cov * dR;
}


Eigen::Matrix3d Link::deriveRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation) {
  Eigen::MatrixXd skewedRotationAxis (rotation.rows(), rotation.cols());
  skewedRotationAxis << 0, -rotationAxis(2), rotationAxis(1),
                rotationAxis(2), 0, -rotationAxis(0),
                -rotationAxis(1), rotationAxis(0), 0;
  return skewedRotationAxis * rotation;
}


void Link::printParameters()
{
  std::cout << "\u03BC = \n" << mean << '\n';
  std::cout << "\u03A3 = \n" << cov << std::endl;
}
