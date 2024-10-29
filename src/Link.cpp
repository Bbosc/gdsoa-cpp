#include "Link.hpp"


Link::Link(const Eigen::Vector2d mean, const Eigen::MatrixXd cov)
  : mMean{mean}, mCov{cov}
{
}

Link::~Link()
{
}

void Link::printParameters()
{
  std::cout << "\u03BC = [";
  for (auto& v: mMean)
  {
    std::cout << v << " ";
  }
  std::cout << "]\n \u03A3 = [";
  for (size_t i{0}; i < mCov.rows(); i++)
  {
    for (size_t j{0}; j < mCov.cols(); j++)
    {
       std::cout << mCov(i, j) << " ";
    }
       std::cout << ";";
  }
  std::cout << "]\n";
}
