#include "distribution.hpp"


double computeMultivariateDistribution(
  const Eigen::Vector3d x, const Eigen::Vector3d mu, const Eigen::Matrix3d sigma)
{
  double prefix = 1/std::sqrt(std::pow(2 * M_PI, x.rows()) * sigma.determinant());
  return prefix * (x-mu).transpose() * sigma.inverse() * (x-mu);
}
