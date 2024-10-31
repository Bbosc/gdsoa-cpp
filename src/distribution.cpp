#include "distribution.hpp"


std::vector<double> linspace(const double start, const double end, const size_t size)
{
  std::vector<double> linearSpace;
  if (start > end){
    std::cout << "start number cannot be greater than end number" << std::endl;
    return linearSpace;
  }
  if (size == 1){
    linearSpace.push_back(start);
    return linearSpace;
  } else if (size == 0) {
    return linearSpace;
  }

  double step { (end-start)/(size-1) };
  for (size_t i {0}; i < size-1; i++)
  {
    linearSpace.push_back(start + step * i);
  }
  linearSpace.push_back(end);
  return linearSpace;
}



double computeMultivariateDistribution(
  const Eigen::Vector3d x, const Eigen::Vector3d mu, const Eigen::Matrix3d sigma)
{
  double prefix = 1/std::sqrt(std::pow(2 * M_PI, x.rows()) * sigma.determinant());
  double exponent = - 0.5 * (x-mu).transpose() * sigma.inverse() * (x-mu);
  return prefix * std::exp(exponent); 
}
