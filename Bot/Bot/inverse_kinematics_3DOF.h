//
// Created by v on 3/26/2023.
//

#ifndef INVERSE_KINEMATICS_3DOF
#define INVERSE_KINEMATICS_3DOF

#include <vector>
#include "robot.h"

class InverseKinematics3DOF {
public:
	static bool calculateIK3DOF(const Manipulator& manipulator, std::vector<double>& jointValues) {
		
		if (manipulator.links.size() != 3 || manipulator.joints.size() != 3) {
			return false; // invalid configuration
		}

		// end-effector position and orientation
		double x = manipulator.endEffector.x;
		double y = manipulator.endEffector.y;
		double z = manipulator.endEffector.z;
		double rx = manipulator.endEffector.rx;
		double ry = manipulator.endEffector.ry;
		double rz = manipulator.endEffector.rz;

		// calculate the inverse kinematics equations based on 3dof all revolute manipulator's geometry

		// joint 1 value calculation
		jointValues[0] = atan2(y, x);

		// joint 2 value calculation
		double l1 = manipulator.links[0].a;
		double l2 = manipulator.links[1].a;

		double j = sqrt(x * x + y * y);
		double k = z - l1;

		double cosTheta2 = (j * j + k * k - l2 * l2) / (2 * j * k);

		// check if cosTheta2 is within the valid range 
		if (cosTheta2 < -1.0 || cosTheta2 > 1.0) {
			return false; // invalid configuration, unable to compute inverse kinematics
		}

		double sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2);

		// check if sinTheta2 is outside the valid range
		if (sinTheta2 < -1.0 || sinTheta2 > 1.0) {
			return false; // invalid configuration, unable to compute inverse kinematics
		}

		jointValues[1] = atan2(k, j) - atan2(sinTheta2, cosTheta2);

		// joint 3 value calculation
		double l3 = manipulator.links[2].a;
		jointValues[2] = rz - jointValues[0] - jointValues[1] - l3;

		return true;
	}
};

//
#endif // INVERSE_KINEMATICS_3DOF
