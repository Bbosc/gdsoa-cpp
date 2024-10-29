#ifndef LINK_HPP
#define LINK_HPP

#include <iostream>
#include <Eigen/Dense>

class Link {
public:

  Link(const Eigen::Vector2d mean, const Eigen::MatrixXd cov);
  ~Link();

  const Eigen::Vector2d getMean() {return mMean;}
  const Eigen::MatrixXd getCov() {return mCov;}

private:
  const Eigen::Vector2d mMean;
  const Eigen::MatrixXd mCov;
};

#endif // !LINK_HPP
