#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <vector>
#include "Link.hpp"


class Robot {
public:

  Robot(const int numberOfJoints = 2);
  ~Robot();

private:
  int mNumberOfJoints;
  std::vector<Link> mLinks;
};

#endif // !ROBOT_HPP
