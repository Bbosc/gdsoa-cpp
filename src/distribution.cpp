#include "distribution.hpp"

namespace dist {

	void setObstaclePosition(Eigen::Vector3d x) {
		obstaclePosition = x;
	}

	double computeMultivariateDistribution(const Eigen::Vector3d mu, const Eigen::Matrix3d sigma) {
	  double prefix = 1/std::sqrt(std::pow(2 * M_PI, obstaclePosition.rows()) * sigma.determinant());
	  double exponent = - 0.5 * (obstaclePosition-mu).transpose() * sigma.inverse() * (obstaclePosition-mu);
	  return prefix * std::exp(exponent); 
	}

	Eigen::Vector3d deriveCollisionMu(const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff) {
		return p * (sigma.inverse() * diff).transpose();
	}

	Eigen::MatrixXd deriveCollisionSigma(const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff){
		Eigen::MatrixXd inv = sigma.inverse();
		return p * 0.5 * (inv * diff * diff.transpose() * inv);
	}

	Eigen::MatrixXd deriveCollision(
			const double p, const Eigen::Matrix3d sigma, const Eigen::Vector3d diff,
			const Eigen::MatrixXd dmudq, const Eigen::MatrixXd dsigmadq) {
		Eigen::MatrixXd dpdmu = deriveCollisionMu(p, sigma, diff);
		Eigen::MatrixXd dpdsigma = deriveCollisionSigma(p, sigma, diff);
		return dpdmu * dmudq +  dpdsigma * dsigmadq;
	}

}
