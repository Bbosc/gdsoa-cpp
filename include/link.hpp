#ifndef LINK_HPP
#define LINK_HPP

#include <iostream>
#include <Eigen/Dense>

class Link {
public:

  Link(const size_t index, const Eigen::Vector3d mean, const Eigen::Matrix3d cov);
  ~Link();

  void updateParameters(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation);
  void printParameters();
  const std::string getName() { return mName; }
  const Eigen::Vector3d getMean() { return mMean; }
  const Eigen::Matrix3d getCov() { return mCov; }

private:
  std::string mName;
  const Eigen::Vector3d mInitialMean;
  const Eigen::Matrix3d mInitialCov;
  Eigen::Vector3d mMean;
  Eigen::Matrix3d mCov;
};

#endif // !LINK_HPP
