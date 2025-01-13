#ifndef LINK_HPP
#define LINK_HPP
#include <iostream>
#include <Eigen/Dense>
#include "distribution.hpp"

typedef Eigen::Vector3d Mean; // GMM mean
typedef Eigen::Vector3d Tra; // 3D translation in task space 
typedef Eigen::Matrix3d Cov; // GMM covariance
typedef Eigen::Matrix3d Rot; // 3D rotation in task space

class Link {
public:
	Link(const Mean, const Cov, const size_t index);
	~Link();
	void updateParameters(const Tra, const Rot, const Eigen::MatrixXd J);
	Eigen::MatrixXd deriveCollision(const Eigen::MatrixXd dR, const Eigen::MatrixXd dT);
	void printParameters();
	const std::string getName() { return name; }

private:
	std::string name;
	const Mean initialMean;
	const Cov initialCov;
	Mean mean;
	Cov cov;
	Rot R, r; // relative and absolute rotations
	Tra T; // translation of link's origin
	Eigen::MatrixXd dT, dR;
	double p;
	
	Eigen::Matrix3d deriveRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation);
	Eigen::Vector3d deriveMu(const Eigen::MatrixXd dR, const Eigen::MatrixXd dT);
	Eigen::Matrix3d deriveSigma(const Eigen::MatrixXd dR);
	Eigen::MatrixXd deriveMu2nd();
	Eigen::MatrixXd deriveSigma2nd();
};

#endif // !LINK_HPP
