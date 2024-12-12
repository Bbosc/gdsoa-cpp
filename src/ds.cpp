#include "ds.hpp"

namespace ds {
	void integrate(Q& q, Q& dq, Q& ddq, double dt) {
		dq = dq + ddq * dt;
		q = q + dq * dt;
	}

	Q computeHarmonic(const Q q, const Q dq, const Q qstar, const Eigen::MatrixXd K, const Eigen::MatrixXd D) {
		return -K * (q - qstar) - D * dq;
	}

	bool isTargetReached(const Q q, const Q target, double tol) {
		return (q - target).norm() < tol;
	}
}
