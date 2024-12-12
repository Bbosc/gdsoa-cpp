#ifndef DS_HPP
#define DS_HPP
#include <Eigen/Dense>

typedef Eigen::Vector2d Q;

namespace ds {

	void integrate(Q& q, Q& dq, Q& ddq, double dt);
	Q computeHarmonic(const Q q, const Q dq, const Q qstar, const Eigen::MatrixXd K, const Eigen::MatrixXd D);
	Q computeGeodesic(const Q q, const Q dq);
	Eigen::MatrixXd computeMetric(const Eigen::MatrixXd dpsi);
	Eigen::MatrixXd computeChristoffel(
		const Eigen::MatrixXd G, const Eigen::MatrixXd dpsi, const Eigen::MatrixXd ddpsi);
	bool isTargetReached(const Q q, const Q target, double tol=0.01);
}


#endif // !DS_HPP
