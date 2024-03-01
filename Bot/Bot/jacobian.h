//
// Created by v on 3/20/2023.
//
//JACOBIAN_H

#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "Robot.h"
#include <Eigen/Dense>

class Jacobian {
public:
	static Eigen::MatrixXd calculateJacobian(const Manipulator& manipulator, double desiredX, double desiredY, double desiredZ, double desiredRX, double desiredRY, double desiredRZ) {
		int dof = manipulator.dof;
		Eigen::MatrixXd jacobian(6, dof);

		// jacobian matrix computation
		for (int i = 0; i < dof; i++) {
			const Manipulator::Joint& joint = manipulator.joints[i];
			const Manipulator::Link& link = manipulator.links[i];

			// joint position
			double q = joint.q;

			// link transformation matrix
			double cos_q = std::cos(q);
			double sin_q = std::sin(q);
			double cos_alpha = std::cos(link.alpha);
			double sin_alpha = std::sin(link.alpha);

			Eigen::Matrix4d linkTransform;
			linkTransform << cos_q, -sin_q * cos_alpha, sin_q * sin_alpha, link.a * cos_q,
				sin_q, cos_q * cos_alpha, -cos_q * sin_alpha, link.a * sin_q,
				0.0, sin_alpha, cos_alpha, link.d,
				0.0, 0.0, 0.0, 1.0;

			// joint axis in the global frame
			Eigen::Vector3d jointAxis;
			jointAxis << linkTransform(0, 2), linkTransform(1, 2), linkTransform(2, 2);

			// joint position in the global frame
			Eigen::Vector3d jointPosition;
			jointPosition << linkTransform(0, 3), linkTransform(1, 3), linkTransform(2, 3);

			// joint velocity cross product matrix
			Eigen::Matrix3d jointCrossMatrix;
			jointCrossMatrix << 0.0, -jointAxis(2), jointAxis(1),
				jointAxis(2), 0.0, -jointAxis(0),
				-jointAxis(1), jointAxis(0), 0.0;

			// linear velocity component of the jacobian
			Eigen::Vector3d desiredPosition(desiredX, desiredY, desiredZ);
			Eigen::Vector3d linearVelocity = jointCrossMatrix * (desiredPosition - jointPosition);

			// angular velocity component of the jacobian
			Eigen::Vector3d angularVelocity = jointAxis;

			// fill the jacobian matrix with linear and angular velocity components
			jacobian.block(0, i, 3, 1) = linearVelocity;
			jacobian.block(3, i, 3, 1) = angularVelocity;
		}

		return jacobian;
	}
};

//
#endif // JACOBIAN_H