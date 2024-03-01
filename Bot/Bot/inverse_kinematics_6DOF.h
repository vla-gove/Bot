// INVERSE_KINEMATICS_6DOF_H

#ifndef INVERSE_KINEMATICS_6DOF
#define INVERSE_KINEMATICS_6DOF

#include "Robot.h" // the header file that defines the Manipulator class
#include <Eigen/Dense>

class InverseKinematics6DOF {
public:
	static bool calculateIK6DOF(const Manipulator& manipulator, Eigen::VectorXd& jointValues) {
		// 6dof check
		if (manipulator.links.size() != 6 || manipulator.joints.size() != 6) {
			return false; // invalid configuration
		}

		// set the initial joint values as the current joint configuration
		jointValues.resize(manipulator.dof);
		for (int i = 0; i < manipulator.dof; i++) {
			jointValues[i] = manipulator.joints[i].q;
		}

		// maximum number of maximum iterations and convergence threshold
		const int maxIterations = 1000;
		const double convergenceThreshold = 1e-3;

		// inverse kinematics calculation using the Newton-Raphson method
		for (int iteration = 0; iteration < maxIterations; iteration++) {
			// forward kinematics to compute the end-effector pose
			Eigen::MatrixXd trans = forwardKinematics(manipulator, jointValues);

			// the error between the desired end-effector pose and the current pose
			Eigen::VectorXd error = computeError(manipulator.endEffector, trans);

			// check if the error is below the convergence threshold
			if (error.norm() < convergenceThreshold) {
				return true; // inverse kinematics calculation successful
			}

			// calculate the Jacobian matrix
			Eigen::MatrixXd jacobian = computeJacobian(manipulator, jointValues);

			// the joint updates using the Newton-Raphson method
			Eigen::VectorXd jointUpdates = jacobian.inverse() * error;

			// update the joint values
			jointValues -= jointUpdates;
		}

		return false; // failed to converge (if it fails it's either unreachable or physically impossible due to the manipulator's configuration)
	}

private:
	static Eigen::MatrixXd forwardKinematics(const Manipulator& manipulator, const Eigen::VectorXd& jointValues) {
		Eigen::MatrixXd transformation = Eigen::MatrixXd::Identity(4, 4);

		// forward kinematics calculation using Denavit-Hartenberg parameters and homogeneous transformations

		for (int i = 0; i < manipulator.dof; i++) {
			double alpha = manipulator.links[i].alpha;
			double a = manipulator.links[i].a;
			double d = manipulator.links[i].d;
			double theta = jointValues[i];

			Eigen::MatrixXd dhTransform(4, 4);
			dhTransform << std::cos(theta), -std::sin(theta) * std::cos(alpha), std::sin(theta) * std::sin(alpha), a * std::cos(theta),
				std::sin(theta), std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha), a * std::sin(theta),
				0, std::sin(alpha), std::cos(alpha), d,
				0, 0, 0, 1;

			transformation *= dhTransform;
		}

		return transformation;
	}

	static Eigen::VectorXd computeError(const Manipulator::EndEffector& desiredPose, const Eigen::MatrixXd& actualPose) {
		Eigen::VectorXd error(6);

		// the error vector between the desired end-effector pose and the actual end-effector pose
		// the vector consists of position error (x, y, z) and orientation error (rx, ry, rz)
		error << desiredPose.x - actualPose(0, 3),
			desiredPose.y - actualPose(1, 3),
			desiredPose.z - actualPose(2, 3),
			desiredPose.rx - std::atan2(actualPose(2, 1), actualPose(2, 2)),
			desiredPose.ry - std::atan2(-actualPose(2, 0), std::sqrt(std::pow(actualPose(2, 1), 2) + std::pow(actualPose(2, 2), 2))),
			desiredPose.rz - std::atan2(actualPose(1, 0), actualPose(0, 0));

		return error;
	}

	static Eigen::MatrixXd computeJacobian(const Manipulator& manipulator, const Eigen::VectorXd& jointValues) {
		Eigen::MatrixXd jac(6, manipulator.dof);

		// calculates the Jacobian matrix using jacobian approximation (numerical differentiation)
		const double delta = 1e-3;

		for (int i = 0; i < manipulator.dof; i++) {
			Eigen::VectorXd jointValuesPlusDelta = jointValues;
			jointValuesPlusDelta[i] += delta;

			Eigen::MatrixXd posePlusDelta = forwardKinematics(manipulator, jointValuesPlusDelta);
			Eigen::MatrixXd pose = forwardKinematics(manipulator, jointValues);

			Eigen::VectorXd deltaPose = Eigen::VectorXd(6);
			deltaPose << posePlusDelta(0, 3) - pose(0, 3),
				posePlusDelta(1, 3) - pose(1, 3),
				posePlusDelta(2, 3) - pose(2, 3),
				std::atan2(posePlusDelta(2, 1), posePlusDelta(2, 2)) - std::atan2(pose(2, 1), pose(2, 2)),
				std::atan2(-posePlusDelta(2, 0), std::sqrt(std::pow(posePlusDelta(2, 1), 2) + std::pow(posePlusDelta(2, 2), 2))) - std::atan2(-pose(2, 0), std::sqrt(std::pow(pose(2, 1), 2) + std::pow(pose(2, 2), 2))),
				std::atan2(posePlusDelta(1, 0), posePlusDelta(0, 0)) - std::atan2(pose(1, 0), pose(0, 0));

			jac.col(i) = deltaPose / delta;
		}

		return jac;
	}
};

//
#endif // INVERSE_KINEMATICS_H
