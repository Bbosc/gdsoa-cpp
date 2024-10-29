#ifndef DISTRIBUTION_HPP
#define DISTRIBUTION_HPP

#include <math.h>
#include <Eigen/Dense>



double computeMultivariateDistribution(
  const Eigen::Vector3d x, const Eigen::Vector3d mu, const Eigen::Matrix3d sigma);




#endif // !DISTRIBUTION_HPP
