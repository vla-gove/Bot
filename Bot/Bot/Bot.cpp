// Bot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "robot.h"
#include "forward_kinematics.h"
#include "inverse_kinematics_3DOF.h"
#include "inverse_kinematics_6DOF.h"
#include "transformation_matrix.h"
#include "jacobian.h"
#include <iostream>
#include <cmath>
#include <vector>

int main() {
	int option;
	while (true) {
		std::cout << "Enter:" << std::endl << "0 for Forward Kinematics calculation" << std::endl
			  <<"1 for Transformation Matrix calculation" << std::endl
			  <<"2 for Inverse Kinematics calculation of a 3DOF manipulator" << std::endl
			  <<"3 for Inverse Kinematics calculation of a 6DOF manipulator" << std::endl
			  <<"4 for Jacobian matrix calculation" << std::endl << std::endl;
		std::cin >> option;
		switch (option) {
		case 0: {
			// forward kinematics calculation: create a Manipulator instance and set the number of degrees of freedom. calculate the end effector position based on the robot's configuration
			Manipulator robot;
			ForwardKinematics fk;

			int dof;
			std::cout << "Enter number of DOF:" << std::endl;
			std::cin >> dof;
			robot.setDof(dof);

			int type;
			double value, length;
			// user-defined joint type and value, and link length for calculating forward kinematics
			for (int i = 0; i < dof; i++) {
				std::cout << "Enter joint type (0 for revolute, 1 for prismatic) for joint " << i << std::endl;
				std::cin >> type;
				robot.setJointType(i, type);

				std::cout << "Enter joint value for joint " << i << " (theta for revolute, d for prismatic)" << std::endl;
				std::cin >> value;
				robot.setJointValue(i, value);

				std::cout << "Enter link length for link " << i << std::endl;
				std::cin >> length;
				robot.setLinkParameters(i, 0, 0, 0, length, 0);
			}

			// calculate forward kinematics
			fk.calculateForwardKinematics(robot);

			// access the end effector position
			double x = robot.endEffector.x;
			double y = robot.endEffector.y;
			double z = robot.endEffector.z;

			// print the end effector position
			std::cout << "End-effector position is: (" << x << ", " << y << ", " << z << ")" <<std::endl << std::endl;
			break;
		}

		case 1: {
			// transformation matrix calculation: create a Manipulator instance, define the parameters, and calculate the transformation matrix based on given robot's parameters
			Manipulator robot;
			int dof;
			std::cout << "Enter number of DOF:" << std::endl;
			std::cin >> dof;
			robot.setDof(dof);

			int type;
			double value, length;
			// user-defined joint type and value, and link length for calculating homogeneous transformation matrices
			for (int i = 0; i < dof; i++) {
				std::cout << "Enter joint type (0 for revolute, 1 for prismatic) for joint " << i << std::endl;
				std::cin >> type;
				robot.setJointType(i, type);

				std::cout << "Enter joint value for joint " << i << " (theta for revolute, d for prismatic):" << std::endl;
				std::cin >> value;
				robot.setJointValue(i, value);

				std::cout << "Enter link length for link " << i << std::endl;
				std::cin >> length;
				robot.setLinkParameters(i, 0, 0, 0, length, 0);
			}

			Transform T;
			T = TransformationMatrix::calculateTransformationMatrix(robot);
			std::cout << std::endl << "Transformation matrix:" << std::endl;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					std::cout << T.data[i][j] << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			break;
		}

		case 2: {
			// inverse kinematics calculation: create a 3dof Manipulator instance, define the parameters, and set the desired end-effector's position and orientation 
			// the given position (x y z) and orientation (r p y) must be reachable for the given configuration

			// create a manipulator object
			Manipulator manipulator;

			// number of degrees of freedom
			manipulator.setDof(3);

			// **MAKE SURE TO PROVIDE REACHABLE AND PHYSICALLY POSSIBLE END EFFECTOR'S POSITION AND ORIENTATION FOR GIVEN ROBOT CONFIGURATION**
			// **WILL NOT COMPUTE IF UNREACHABLE OR PHYSICALLY IMPOSSIBLE**

			// joint types (0 for revolute, 1 for prismatic)
			manipulator.setJointType(0, 0); // q1 is revolute
			manipulator.setJointType(1, 0); // q2 is revolute
			manipulator.setJointType(2, 0); // q3 is revolute

			// link parameters
			manipulator.setLinkParameters(0, 0.0, 0.0, 0.0, 2.0, 0.0);
			manipulator.setLinkParameters(1, 0.0, 0.0, 0.0, 3.0, 0.0);
			manipulator.setLinkParameters(2, 0.0, 0.0, 0.0, 2.5, 0.0);

			// end-effector position
			manipulator.setEndEffectorPosition(4, 5, 6); // x y z 

			// end-effector orientation 
			manipulator.setEndEffectorOrientation(0, 0, 0); // rx ry rz in radians

			// call the inverse kinematics function
			std::vector<double> jointValues(3); // store the joint values
			bool success = InverseKinematics3DOF::calculateIK3DOF(manipulator, jointValues);

			if (success) {
				// print the calculated joint values
				std::cout << "Inverse Kinematics solution:" << std::endl;
				std::cout << "q1: " << jointValues[0] << " radians" << std::endl;
				std::cout << "q2: " << jointValues[1] << " radians" << std::endl;
				std::cout << "q3: " << jointValues[2] << " radians" << std::endl << std::endl;
			}
			else {
				std::cout << "Inverse Kinematics calculation failed. Is the position reachable and physically possible?" << std::endl << std::endl;
			}
			break;
		}

		case 3: {
			// inverse kinematics calculation: create a 6dof Manipulator instance and its configuration, and calculate inverse kinematics based on the given end effector position and orientation
			// the given position (x y z) and orientation (r p y) must be reachable for the given configuration

			// ***ALL THE ROBOT CONFIGURATION PARAMETERS AND END EFFECTOR'S POSITION AND ORIENTATION ARE SET TO ZERO***
			// ***USE REACHABLE AND PHYSICALLY POSSIBLE END EFFECTOR'S POSITION AND ORIENTATION FOR SET CONFIGURATION***

			// create a Manipulator instance and set its parameters
			Manipulator manipulator;
			manipulator.setDof(6);

			// set the parameters of the links and joints
			manipulator.setJointType(0, 0); // q1 is revolute
			manipulator.setJointType(1, 0); // q2 is revolute
			manipulator.setJointType(2, 0); // q3 is revolute
			manipulator.setJointType(3, 0); // q4 is revolute
			manipulator.setJointType(4, 0); // q5 is revolute
			manipulator.setJointType(5, 0); // q6 is revolute

			manipulator.setLinkParameters(0, 0.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(1, 0.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(2, 0.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(3, 0.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(4, 0.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(5, 0.0, 0.0, 0.0, 0.0, 0.0);

			// set the desired end-effector pose
			Manipulator::EndEffector desiredPose;
			desiredPose.x = 0;
			desiredPose.y = 0;
			desiredPose.z = 0;
			desiredPose.rx = 0;
			desiredPose.ry = 0;
			desiredPose.rz = 0;
			manipulator.setEndEffectorPosition(desiredPose.x, desiredPose.y, desiredPose.z);
			manipulator.setEndEffectorOrientation(desiredPose.rx, desiredPose.ry, desiredPose.rz);

			// call the inverse kinematics calculation function
			Eigen::VectorXd jointValues;
			bool complete = InverseKinematics6DOF::calculateIK6DOF(manipulator, jointValues);

			// check if the inverse kinematics calculation was successful
			if (complete) {
				// print the resulting joint values
				std::cout << "Inverse Kinematics solution:" << std::endl;
				for (int i = 0; i < jointValues.size(); i++) {
					std::cout << "q" << i << ": " << jointValues[i] << " radians" << std::endl;
				}
				std::cout << std::endl;
			}
			else {
				std::cout << "Inverse Kinematics calculation failed. Is the position reachable and physically possible?" << std::endl;
			}

			break;
		}
		
		case 4: {
			// jacobian matrix calculation: define a manipulator instance and its configuration (assumes all revolute joints, 6DOF), calculate the jacobian matrix based on the desired end effector's position and orientation

			// create a manipulator object
			Manipulator manipulator;
			manipulator.setDof(6);

			// joint types
			manipulator.setJointType(0, 0); // all revolute joints
			manipulator.setJointType(1, 0);
			manipulator.setJointType(2, 0);
			manipulator.setJointType(3, 0);
			manipulator.setJointType(4, 0);
			manipulator.setJointType(5, 0);

			// joint values
			manipulator.setJointValue(0, 0.5);
			manipulator.setJointValue(1, 0.3);
			manipulator.setJointValue(2, 0.2);
			manipulator.setJointValue(3, 0.1);
			manipulator.setJointValue(4, 0.4);
			manipulator.setJointValue(5, 0.6);

			// link parameters
			manipulator.setLinkParameters(0, 1.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(1, 1.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(2, 1.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(3, 1.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(4, 1.0, 0.0, 0.0, 0.0, 0.0);
			manipulator.setLinkParameters(5, 1.0, 0.0, 0.0, 0.0, 0.0);

			// desired end effector pose
			double desiredX = 0.5;
			double desiredY = 0.3;
			double desiredZ = 0.2;
			double desiredRX = 0.1;
			double desiredRY = 0.4;
			double desiredRZ = 0.6;

			// calculate the jacobian
			Eigen::MatrixXd jacobian = Jacobian::calculateJacobian(manipulator, desiredX, desiredY, desiredZ, desiredRX, desiredRY, desiredRZ);

			// print the jacobian
			std::cout << "Calculated Jacobian matrix:" << std::endl;
			std::cout << jacobian << std::endl << std::endl;

			break;
		}
		}
	}
}
