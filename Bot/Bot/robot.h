// ROBOT_H

#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <vector>
#include <algorithm>

class Manipulator {
public:
	class Link {
	public:
		double mass;
		double inertia;
		double alpha; // denavit hartenberg parameter: link twist angle
		double a;     // dh parameter: link length
		double d;     // dh parameter: link offset
		// other link parameters
	};

	class Joint {
	public:
		int type;      // 0 for revolute, 1 for prismatic
		double q;      // joint variable
		double dq;     // joint velocity
		double ddq;    // joint acceleration
		double torque; // joint torque
		// other joint parameters
	};

	class EndEffector {
	public:
		double x;  // end-effector position in x-axis
		double y;  // end-effector position in y-axis
		double z;  // end-effector position in z-axis
		double rx; // end-effector orientation in x-axis
		double ry; // end-effector orientation in y-axis
		double rz; // end-effector orientation in z-axis
	};

	int dof;  // number of degrees of freedom
	std::vector<Link> links;
	std::vector<Joint> joints;
	EndEffector endEffector;
	// other manipulator parameters

	void setDof(int dof) {
		this->dof = dof;
		links.resize(dof);
		joints.resize(dof);
	}

	void setJointType(int index, int type) {
		joints[index].type = type;
	}

	void setJointValue(int index, double q) {
		joints[index].q = q;
	}

	void setJointVelocity(int index, double dq) {
		joints[index].dq = dq;
	}

	void setJointAcceleration(int index, double ddq) {
		joints[index].ddq = ddq;
	}

	void setJointTorque(int index, double torque) {
		joints[index].torque = torque;
	}

	void setLinkParameters(int index, double mass, double inertia, double alpha, double a, double d) {
		links[index].mass = mass;
		links[index].inertia = inertia;
		links[index].alpha = alpha;
		links[index].a = a;
		links[index].d = d;
	}

	void setEndEffectorPosition(double x, double y, double z) {
		endEffector.x = x;
		endEffector.y = y;
		endEffector.z = z;
	}

	void setEndEffectorOrientation(double rx, double ry, double rz) {
		endEffector.rx = rx;
		endEffector.ry = ry;
		endEffector.rz = rz;
	}
};

#endif //ROBOT_H
