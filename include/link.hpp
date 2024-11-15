#ifndef LINK_HPP
#define LINK_HPP

#include <iostream>
#include <Eigen/Dense>

class Link {
public:

  Link(const size_t index, const Eigen::Vector3d mean, const Eigen::Matrix3d cov);
  ~Link();

  void updateParameters(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation, const Eigen::MatrixXd J, const Eigen::MatrixXd localRotation);
  Eigen::MatrixXd derivateLocalRotation(const Eigen::Vector3d rotationAxis, const Eigen::MatrixXd rotation);
  void printParameters();
  const std::string getName() { return mName; }
  const Eigen::Vector3d getMean() { return mMean; }
  const Eigen::Matrix3d getCov() { return mCov; }
  const size_t getIndex() { return mIndex; }

private:
  std::string mName;
  const Eigen::Vector3d mInitialMean;
  const Eigen::Matrix3d mInitialCov;
  Eigen::Vector3d mMean;
  Eigen::Matrix3d mCov;
  const size_t mIndex;
};

#endif // !LINK_HPP
