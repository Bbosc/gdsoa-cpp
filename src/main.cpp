#include <math.h>
#include "robot.hpp"
#include "distribution.hpp"
#include "file_manager.hpp"
#include "ds.hpp"


std::vector<Eigen::Vector3d> computeProbaMap(std::vector<double>, Robot, Eigen::Vector3d);


int main (int argc, char *argv[]) {

	double minAngle {-M_PI/3};
	double maxAngle {M_PI/3};
	size_t resolution {5};
	std::vector<double> angles {linspace(minAngle, maxAngle, resolution)};

	Robot robot(2, std::string("/home/baudouin/Repositories/gdsoa-cpp/description/2_joints_arm.urdf"));

	Eigen::Vector3d obstaclePosition(-0.5, 1.8, 0);
	// std::vector<Eigen::Vector3d> probaMap = computeProbaMap(angles, robot, obstaclePosition);
	// writeVectorsToFile<Eigen::Vector3d>("test.csv", probaMap);
	// ds to generate a trajectory
	Q q(0.0, M_PI/5);
	Q dq(0.0, 0.0);
	Q ddq;
	Q target(0.2, 0.8);
	double dt = 0.1;
	Eigen::MatrixXd K(q.size(), q.size());
	K << 0.5 * Eigen::MatrixXd::Identity(q.size(), q.size());
	Eigen::MatrixXd D(q.size(), q.size());
	D << 1.5 * Eigen::MatrixXd::Identity(q.size(), q.size());
	while (!ds::isTargetReached(q, target)) {
		ddq = ds::computeHarmonic(q, dq, target, K, D);
		ds::integrate(q, dq, ddq, dt);
	}
	return 0;
}


std::vector<Eigen::Vector3d> computeProbaMap(std::vector<double> angles, Robot robot, Eigen::Vector3d obstacle) {
	std::vector<Eigen::Vector3d> probabilityMap;
	for (size_t i{0}; i < angles.size(); i++) {
		for (size_t j{0}; j < angles.size(); j++) {   
			Eigen::Vector2d q(angles[i], angles[j]);
			robot.move(q);
			double collisionProbability {0};
		  	for (size_t k{0}; k < robot.getLinks().size(); k++) {
				collisionProbability += computeMultivariateDistribution(
					obstacle, robot.getLinks()[k].getMean(), robot.getLinks()[k].getCov()); 
		  	}
			Eigen::Vector3d location (angles[i], angles[j], collisionProbability);
			probabilityMap.push_back(location);
		}
	}
	return probabilityMap;
}
