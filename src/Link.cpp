#include "Link.hpp"


Link::Link(const size_t index, const Eigen::Vector3d mean, const Eigen::Matrix3d cov)
  : mInitialMean{mean}, mInitialCov{cov}, mMean{mean}, mCov{cov}
{
  mName = std::string("link") + std::to_string(index);
}

Link::~Link()
{
}

void Link::updateParameters(const Eigen::Vector3d translation, const Eigen::Matrix3d rotation)
{
  Eigen::Vector3d updatedMean = translation + rotation * mInitialMean;
  for (size_t i{0}; i < updatedMean.rows(); i++)
  {
    mMean[i] = updatedMean[i];
  }
  Eigen::MatrixXd updatedCov = rotation * mInitialCov * rotation.transpose();
  for (size_t i{0}; i < updatedCov.rows(); i++)
  {
    for (size_t j{0}; j < updatedCov.cols(); j++)
    {
      mCov(i, j) = updatedCov(i, j);
    }
  }


}


void Link::printParameters()
{
  std::cout << "\u03BC = \n" << mMean << '\n';
  std::cout << "\u03A3 = \n" << mCov << std::endl;
}
