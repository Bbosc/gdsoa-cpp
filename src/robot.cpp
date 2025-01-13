#include "robot.hpp"


Robot::Robot(const int numberOfJoints, const std::string urdfFile) 
	: numberOfJoints{numberOfJoints}
{
	pinocchio::urdf::buildModel(urdfFile, model);
	pinocchio::Data data(model);
	this->data = data;


	const Eigen::Vector3d mean0(0, 0.5, 0);
	Eigen::Matrix3d cov0(3, 3);
	cov0(0,0) = 0.02;
	cov0(1,0) = -0.0004;
	cov0(2,0) = -0.0002;
	cov0(0,1) = cov0(1,0);
	cov0(1,1) = 0.0839;
	cov0(2,1) = 0.001;
	cov0(0,2) = cov0(2,0);
	cov0(1,2) = cov0(2,1);
	cov0(2,2) = 0.021;

	for (size_t i = 0; i < numberOfJoints; i++)
	{
		Link link(mean0, cov0, i+1);
		links.push_back(link);
	}
  
}

Robot::~Robot() 
{
}

void Robot::displayLinks()
{
	for (size_t i {0}; i < numberOfJoints; i++)
	{
		std::cout << "Link " << i << ":\n";
		links[i].printParameters();
	}
}

void Robot::move(const Eigen::VectorXd q)
{
	pinocchio::forwardKinematics(model, data, q);
	pinocchio::updateFramePlacements(model, data);
	size_t frameId;
	int index = 0;
	Eigen::Vector3d translation;
	Rot R, dR; 
	Eigen::MatrixXd J(6, model.nv);
	Eigen::MatrixXd dp;
	for (Link& link: links) {
		frameId = model.getFrameId(link.getName());
		translation = data.oMf[frameId].translation();
		R = data.oMf[frameId].rotation();
		pinocchio::computeFrameJacobian(model, data, q, frameId, pinocchio::LOCAL, J); 
		link.updateParameters(translation, R, J);
		dR = deriveRotation(frameId);
		dp = link.deriveCollision(dR, J.block<3, 1>(0, index));
		index++;
	}
}

void Robot::moveToTarget(
		Q q, const Q target, const Eigen::MatrixXd K, const Eigen::MatrixXd D, const double dt) {
	Q dq(0.0, 0.0);
	Q ddq;
	Eigen::MatrixXd dp, ddp;
	while (!ds::isTargetReached(q, target)) {
		ddq = ds::computeHarmonic(q, dq, target, K, D);
		/*ddq += ds::computeGeodesic(dq, dp, ddp);*/
		ds::integrate(q, dq, ddq, dt);
		move(q);
	}
}

Rot Robot::getRelativeRotation(size_t frameId) {
	int parentFrameId = model.frames[frameId].parent;
	pinocchio::SE3 toBase = data.oMf[frameId];
	pinocchio::SE3 parentToBase = data.oMf[parentFrameId];
	pinocchio::SE3 relativeTransform = toBase * parentToBase.inverse();
	return relativeTransform.rotation();
}

Eigen::Vector3d Robot::getRotationAxis(size_t frameId) {
	Eigen::Vector3d angularV = data.v[frameId].angular();
	return angularV.normalized();
}

Eigen::MatrixXd Robot::deriveRelativeRotation(size_t frameId, Rot r) {
	Eigen::Vector3d axis = getRotationAxis(frameId);
	Eigen::MatrixXd skewedRotationAxis (r.rows(), r.cols());
	skewedRotationAxis << 0, -axis(2), axis(1),
						axis(2), 0, -axis(0),
						-axis(1), axis(0), 0;
	return skewedRotationAxis * r;
}

Eigen::MatrixXd Robot::deriveRotation(size_t frameId) {
	Rot r = getRelativeRotation(frameId);
	Eigen::MatrixXd dr = deriveRelativeRotation(frameId, r);
	Rot dR = dr;
	size_t parentId = frameId;
	do {
		parentId = model.frames[parentId].parent;
		dR *= data.oMf[parentId].rotation();
	} while (parentId != 0);
	return dR; 
}

void Robot::updateCollisionGradient(Eigen::MatrixXd& dp){
}
