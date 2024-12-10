#ifndef LINK_HPP
#define LINK_HPP

#include <iostream>
#include <Eigen/Dense>

typedef Eigen::Vector3d Mean; // GMM mean
typedef Eigen::Vector3d Tra; // 3D translation in task space 
typedef Eigen::Matrix3d Cov; // GMM covariance
typedef Eigen::Matrix3d Rot; // 3D rotation in task space

class Link {
public:
	Link(const size_t index, const Mean, const Cov);
	~Link();
	void updateParameters(const Tra, const Rot);
	Eigen::MatrixXd derivateMu(const Eigen::MatrixXd dR, const Mean, const Eigen::MatrixXd dT);
	Eigen::MatrixXd derivateSigma(const Rot, const Eigen::MatrixXd dR, const Cov);
	Eigen::MatrixXd derivateMu2nd();
	Eigen::MatrixXd derivateSigma2nd();

	void printParameters();

	const std::string getName() { return mName; }
	const Mean getMean() { return mMean; }
	const Cov getCov() { return mCov; }
	const size_t getIndex() { return mIndex; }

private:
	std::string mName;
	const Mean mInitialMean;
	const Cov mInitialCov;
	Mean mMean;
	Cov mCov;
	const size_t mIndex;
	
	Eigen::MatrixXd derivateRelativeRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation);
};

#endif // !LINK_HPP
