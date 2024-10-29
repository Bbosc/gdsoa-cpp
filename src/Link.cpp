#include "Link.hpp"


Link::Link(const Eigen::Vector2d mean, const Eigen::MatrixXd cov)
  : mMean{mean}, mCov{cov}
{
  std::cout << "creating a link\n";
}

Link::~Link()
{
  std::cout << "destroying link ..." << std::endl;
}
