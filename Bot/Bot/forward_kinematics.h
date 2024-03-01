//
// Created by v on 1/26/2023.
//
//FORWARD_KINEMATICS_H

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <cmath>
#include <vector>
#include "robot.h"
#include <Eigen/Dense>

class ForwardKinematics {
public:
	void calculateForwardKinematics(Manipulator& robot) {
		// initialize transformation matrix
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

		// iterate through each joint to calculate individual transformation matrices
		for (int i = 0; i < robot.dof; i++) {
			const Manipulator::Link& link = robot.links[i];
			const Manipulator::Joint& joint = robot.joints[i];

			// dh parameters
			double alpha = link.alpha;
			double a = link.a;
			double d = link.d;
			double theta = joint.q;

			// calculate transformation matrix for joint i
			Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
			A << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
				sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),              
				0, sin(alpha), cos(alpha), d,
				0, 0, 0, 1;

			// update the overall transformation matrix
			T = T * A;
		}

		// the end effector position from the transformation matrix
		double x = T(0, 3);
		double y = T(1, 3);
		double z = T(2, 3);

		// update the robot's end effector position
		robot.setEndEffectorPosition(x, y, z);
	}
};

//
#endif //FORWARD_KINEMATICS_H