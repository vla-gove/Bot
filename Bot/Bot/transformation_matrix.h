//
// Created by v on 1/28/2023.
//

#ifndef TRANSFORMATRION_MATRIX_H
#define TRANSFORMATION_MATRIX_H

#include <cmath>
#include <vector>
#include "robot.h"

// 4x4 transformation matrix
class Transform {
public:
	double data[4][4];
	Transform() {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				data[i][j] = 0.0;
			}
		}
	}
};

class TransformationMatrix {
public:
	static Transform calculateTransformationMatrix(Manipulator robot) {
		// identity matrix initialization
		Transform T;
		T.data[0][0] = 1;
		T.data[1][1] = 1;
		T.data[2][2] = 1;
		T.data[3][3] = 1;

		// iterate through each individual joint
		for (int i = 0; i < robot.dof; i++) {
			Transform T_i;
			if (robot.joints[i].type == 0) { // for revolute joints
				double c = cos(robot.joints[i].q);
				double s = sin(robot.joints[i].q);

				T_i.data[0][0] = c;
				T_i.data[0][1] = -s;
				T_i.data[1][0] = s;
				T_i.data[1][1] = c;
				T_i.data[0][3] = robot.links[i].a;
				T_i.data[1][3] = 0;
				T_i.data[2][3] = 0;
				T_i.data[3][3] = 1;
			}
			else { // for prismatic joints
				T_i.data[0][0] = 1;
				T_i.data[1][1] = 1;
				T_i.data[2][2] = 1;
				T_i.data[3][3] = 1;
				T_i.data[2][3] = robot.joints[i].q;
			}

			// multiply the current transformation matrix with the individual joint transformation matrix
			T = multiply(T, T_i);
		}

		return T;
	}

	// matrix multiplication function
	static Transform multiply(Transform A, Transform B) {
		Transform C;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					C.data[i][j] += A.data[i][k] * B.data[k][j];
				}
			}
		}
		return C;
	}
};

//
#endif //TRANSFORMATION_MATRIX_H
