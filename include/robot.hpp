#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <vector>
#include <string>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "link.hpp"
#include "ds.hpp"


class Robot {
public:

  Robot(const int numberOfJoints, const std::string urdfFile);
  ~Robot();

  void displayLinks();

  void move(const Eigen::VectorXd jointConfiguration);
  void moveToTarget(Q q, const Q target, const Eigen::MatrixXd K, const Eigen::MatrixXd D, const double dt);
  std::vector<Link> getLinks() { return links; }

private:
  int numberOfJoints;
  std::vector<Link> links;
  pinocchio::Model model;
  pinocchio::Data data;
  Rot getRelativeRotation(size_t frameId);
  Eigen::Vector3d getRotationAxis(size_t frameId);
  Eigen::MatrixXd deriveRelativeRotation(size_t frameId, Rot r);
  Eigen::MatrixXd deriveRotation(size_t frameId);
  void updateCollisionGradient(Eigen::MatrixXd&);
  void updateCollisionHessian(Eigen::MatrixXd&);
};


#endif // !ROBOT_HPP
