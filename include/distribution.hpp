#ifndef DISTRIBUTION_HPP
#define DISTRIBUTION_HPP

#include <iostream>
#include <Eigen/Dense>


namespace dist {
	static Eigen::Vector3d obstaclePosition;
	void setObstaclePosition(Eigen::Vector3d x);
	double computeMultivariateDistribution(const Eigen::Vector3d mu, const Eigen::Matrix3d sigma);
	Eigen::Vector3d deriveCollisionMu(const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff);
	Eigen::MatrixXd deriveCollisionSigma(const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff);
	Eigen::MatrixXd deriveCollision(
			const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff,
			const Eigen::MatrixXd dmudq, const Eigen::MatrixXd dsigmadq);
}


#endif // !DISTRIBUTION_HPP
