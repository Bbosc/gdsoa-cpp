#ifndef DISTRIBUTION_HPP
#define DISTRIBUTION_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>


std::vector<double> linspace(const double start, const double end, const size_t size);

double computeMultivariateDistribution(
  const Eigen::Vector3d x, const Eigen::Vector3d mu, const Eigen::Matrix3d sigma);




#endif // !DISTRIBUTION_HPP
