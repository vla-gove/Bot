//
// Created by v on 1/26/2023.
//
//FORWARD_KINEMATICS_H
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <cmath>
#include <vector>
#include "robot.h"

class ForwardKinematics {
public:
	std::vector<double> calculateFK(Manipulator& robot) {
		std::vector<double> endEffectorPos;
		double x = 0, y = 0, z = 0;
		double alpha = 0, beta = 0, gamma = 0;
		for (int i = 0; i < robot.dof; i++) {
			//calculation for revolute joints
			if (robot.joints[i].type == 0) {
				alpha += robot.joints[i].q;
				x += robot.links[i].length * cos(alpha);
				y += robot.links[i].length * sin(alpha);
			}
			//calculation for prismatic joints
			else if (robot.joints[i].type == 1) {
				z += robot.joints[i].q;
			}
			beta += robot.joints[i].q;
			gamma += robot.joints[i].q;
		}
		endEffectorPos.push_back(x);
		endEffectorPos.push_back(y);
		endEffectorPos.push_back(z);
		endEffectorPos.push_back(alpha);
		endEffectorPos.push_back(beta);
		endEffectorPos.push_back(gamma);
		return endEffectorPos;
	}
};

#endif //FORWARD_KINEMATICS_H