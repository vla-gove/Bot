#include "robot.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "transformation_matrix.h"
#include <iostream>
#include <cmath>

int main() {
    int choice;
    while (true) {
        std::cout << "what do you want to do?\n(0 for forward kinematics computation, 1 for transformation matrix computation, 2 for inverse kinematics computation)\n";
        std::cin >> choice;
        switch (choice) {
            case 0: {
                Manipulator robot;
                int dof;
                std::cout << "enter number of dof:\n";
                std::cin >> dof;
                robot.setDof(dof);

                int type;
                double value, length;
                //defining joint types and values, and link length for calculating forward kinematics
                for (int i = 0; i < dof; i++) {
                    std::cout << "enter joint type (0 for revolute, 1 for prismatic) for joint:\n";
                    std::cin >> type;
                    robot.setJointType(i, type);

                    std::cout << "enter joint value for joint (theta for revolute, d for prismatic:\n";
                    std::cin >> value;
                    robot.setJointValue(i, value);

                    std::cout << "enter link length for link:\n";
                    std::cin >> length;
                    robot.setLinkLength(i, length);
                }
                //robot parameters such as mass and inertia are not considered when calculating forward kinematics
                ForwardKinematics fk;
                std::vector<double> pos = fk.calculateFK(robot);
                std::cout << "x y z end effector position is: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n"
                          << std::endl;
                break;
            }

            case 1: {
                Manipulator robot;
                int dof;
                std::cout << "enter number of dof:\n";
                std::cin >> dof;
                robot.setDof(dof);

                int type;
                double value, length;
                //user defined joint type and value, and link length for calculating homogenous transformation matrices
                for (int i = 0; i < dof; i++) {
                    std::cout << "enter joint type (0 for revolute, 1 for prismatic) for joint:\n";
                    std::cin >> type;
                    robot.setJointType(i, type);

                    std::cout << "enter joint value for joint (theta for revolute, d for prismatic:\n";
                    std::cin >> value;
                    robot.setJointValue(i, value);

                    std::cout << "enter link length for link:\n";
                    std::cin >> length;
                    robot.setLinkLength(i, length);
                }
                Transform T;
                T = TransformationMatrix::calculateTransformationMatrix(robot);
                std::cout << "Transformation matrix:" << std::endl;
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        std::cout << T.data[i][j] << " ";
                    }
                    std::cout << std::endl;
                }
                // std::cout << "this functionality is not yet added or implemented!\n";
                break;
            }


            case 2:
                std::cout << "this functionality is not yet added or implemented!\n";
                break;


        }
    }
}