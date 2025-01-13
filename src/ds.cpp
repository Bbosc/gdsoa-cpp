#include "ds.hpp"

namespace ds {
	void integrate(Q& q, Q& dq, Q& ddq, double dt) {
		dq = dq + ddq * dt;
		q = q + dq * dt;
	}

	Q computeHarmonic(
		const Q q, const Q dq, const Q qstar, const Eigen::MatrixXd K, const Eigen::MatrixXd D) {
		return -K * (q - qstar) - D * dq;
	}

/*
	Q computeGeodesic(const Q dq, const Eigen::VectorXd dpsi, const Eigen::MatrixXd ddpsi) {
		Eigen::MatrixXd G = computeMetric(dpsi);
		Eigen::MatrixXd dG = deriveMetric(dpsi, ddpsi);
		Eigen::MatrixXd Xi = computeChristoffel(G, dG, dpsi, ddpsi);
		return Xi * dq * dq;
	}

	Eigen::MatrixXd computeChristoffel(
		const Eigen::MatrixXd G, const Eigen::MatrixXd dG, const Eigen::MatrixXd dpsi, const Eigen::MatrixXd ddpsi) {
		Eigen::MatrixXd iG = G.inverse();
		return 0.5 * (iG * (dG + dG.transpose()) - iG * dG);
	}

	Eigen::MatrixXd deriveMetric(const Eigen::MatrixXd dpsi, const Eigen::MatrixXd ddpsi) {
		return ddpsi * dpsi + dpsi * ddpsi;
	}

	Eigen::MatrixXd computeMetric(const Eigen::MatrixXd dpsi) {
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dpsi.rows(), dpsi.rows());
		return I + dpsi.cross(dpsi);
	}

*/
	bool isTargetReached(const Q q, const Q target, double tol) {
		return (q - target).norm() < tol;
	}
}
