#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <vector>
#include <string>
#include "Link.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/frames.hpp"


class Robot {
public:

  Robot(const int numberOfJoints, const std::string urdfFile);
  ~Robot();

  void displayLinks();

  void move(const Eigen::VectorXd jointConfiguration);

private:
  int mNumberOfJoints;
  std::vector<Link> mLinks;
  pinocchio::Model mModel;
  pinocchio::Data mData;
};

#endif // !ROBOT_HPP
