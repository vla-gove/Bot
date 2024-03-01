# Bot

A C++ Console App for Robot Manipulators' kinematic parameters analysis. Includes Forward Kinematics, Homogenous Transformation Matrix, Inverse Kinematics, and Jacobian Matrix computation.

# Forward Kinematics computation
Calculates Forward Kinematics of a Robot Manipulator with an arbitrary configuration (user-input number of degrees of freedom, joint types and values, and link lengths). Forward kinematics calculates the position and orientation of the end-effector for given joint values. Degrees of freedom, joint types and values, and link lengths are user-input through the console.

![FK](https://github.com/vla-gove/Bot/assets/107414426/88609f47-8a6d-4b23-888b-458ae00ff667)

# Homogenous Transformation Matrices computation
Calculates a Homogenous Transformation Matrix of a Robot Manipulator with an arbitrary configuration (user-input number of degrees of freedom, joint types and values, and link lengths). A homogenous transformation matrix describes the position and orientation of the end-effector with respect to the reference coordinate frame. Degrees of freedom, joint types and values, and link lengths are user-input through the console.

![HTM](https://github.com/vla-gove/Bot/assets/107414426/0bda33df-a9af-42cd-904e-1f5c08c7037d)

# Inverse Kinematics computation
Calculates Inverse Kinematics of a 3-DOF or a 6-DOF Robot Manipulator with all revolute joints. Inverse kinematics calculates the joint values (theta) necessary to achieve the set end-effector position and orientation. Robot Manipulator's configuration parameters and the desired end-effector position and orientation are set within the program itself. Make sure to use physically possible and reachable end-effector's position and orientation in order to successfully calculate inverse kinematics.

![IK3DOF](https://github.com/vla-gove/Bot/assets/107414426/b1698d20-c172-4133-8365-95c91c692524)

![IK6DOF](https://github.com/vla-gove/Bot/assets/107414426/4f38a226-a9bf-4b2c-974a-1f71c809e050)

# Jacobian Matrix Computation
Calculates the Jacobian Matrix for a 6-DOF Robot Manipulator configuration. The Jacobian matrix relates the velocity of the end-effector to joint velocities. Robot Manipulator's configuration parameters and the desired end-effector position and orientation are set within the program itself.

![JAC](https://github.com/vla-gove/Bot/assets/107414426/773228a2-ea34-4f6f-b658-c3a0472156d1)


